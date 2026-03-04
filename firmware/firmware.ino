/*
 * GSR Touch Detector + PWM Pump "breath" + LED feedback – 20 Hz
 * Identico schema ADC→µS del logger di acquisizione.
 * Pipeline: LP 0.5 Hz (Butter 2°), SCL EMA τ=10 s, detector, feature 0–3 s, RF micromlgen.
 * Filtro anti-artefatti opzionale (ENABLE_ARTIFACT_FILTER).
 * Plot grezzo µS opzionale (ENABLE_PLOT).
 */

#include <math.h>
#ifndef INFINITY
  #define INFINITY (1.0f/0.0f)
#endif
#define inf INFINITY
#include "rf_touch.h"

// ===== SWITCH =====
bool ENABLE_ARTIFACT_FILTER = true;  // ON/OFF filtro anti-artefatti
bool ENABLE_PLOT            = true; // ON/OFF stampa µS per plotting

// ===== PIN & CALIBRAZIONE =====
const int GSR_PIN = 32;
const int SERIAL_CALIBRATION = 2475;
const int YELLOW_LED = 27;
const int RED_LED    = 14;
const int GREEN_LED  = 12;

// ===== PARAMETRI DETECTOR =====
const float FS = 20.0f, DT = 1.0f / 20.0f;
const float K_SIGMA = 1.5f, A_MIN_US = 0.6f, REFRACT_S = 6.0f;
const int PRE_SAMPLES=int(3.0f*FS), POST_SAMPLES=int(8.0f*FS),
          POST3_SAMPLES=int(3.0f*FS), PRE2_SAMPLES=int(1.5f*FS),
          MAX_BUF=PRE_SAMPLES+POST_SAMPLES;
const float TAU_SCL=10.0f, ALPHA_SCL=1.0f-expf(-1.0f/(TAU_SCL*FS));

// ===== FILTRO LP 0.5 Hz =====
struct Biquad {
  float b0,b1,b2,a1,a2,x1,x2,y1,y2;
  void set(float cb0,float cb1,float cb2,float ca1,float ca2){
    b0=cb0; b1=cb1; b2=cb2; a1=ca1; a2=ca2; x1=x2=y1=y2=0;
  }
  float step(float x){
    float y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2;
    x2=x1; x1=x; y2=y1; y1=y; return y;
  }
};
Biquad lp;

// ===== ROLLING SIGMA(d) =====
#define SIGMA_WIN (60*20)
float d_ring[SIGMA_WIN]; int d_idx=0,d_count=0;
inline void sigma_push(float v){ d_ring[d_idx]=v; d_idx=(d_idx+1)%SIGMA_WIN; if(d_count<SIGMA_WIN)d_count++; }
inline float rolling_sigma(){
  if(!d_count) return 1e-3f;
  float m=0; for(int i=0;i<d_count;i++) m+=d_ring[i]; m/=d_count;
  float v=0; for(int i=0;i<d_count;i++){ float e=d_ring[i]-m; v+=e*e; }
  v/=(d_count>1?d_count-1:1); return sqrtf(fmaxf(v,1e-8f));
}

// ===== BUFFER SCR/SCL =====
float scr_buf[MAX_BUF], scl_buf[MAX_BUF]; int buf_len=0;

// ===== STATI DETECTOR =====
enum State { IDLE, WAIT_POST, REFRACT };
State state=IDLE; int onset_pos_in_buf=-1, refract_left=0;

// ===== SOGLIE ANTI-ARTEFATTI =====
const float MIN_TPEAK_S=0.40f;
const float MIN_WIDTH_HALF_S=0.30f;
const float MAX_SLOPE_US_PER_S=3.5f;
const float MIN_AUC_US_S=0.03f;

// ===== CLASSIFICATORE =====
Eloquent::ML::Port::RandomForest clf;

// ===== LED STATE =====
bool LED_Y=false, LED_R=false, LED_G=false;
inline void leds_apply(){
  digitalWrite(YELLOW_LED,LED_Y);
  digitalWrite(RED_LED,LED_R);
  digitalWrite(GREEN_LED,LED_G);
}

// ===== POMPA PWM =====
const int PIN_PUMP=25, PWM_CH=0, PWM_FREQ=2000, PWM_RES=8;
const uint8_t DUTY_MIN=70, DUTY_MAX=220;
const uint32_t BREATH_PERIOD_MS=3000; const int BREATH_CYCLES=3;
enum PumpState{P_IDLE,P_BREATH}; PumpState pstate=P_IDLE;
uint32_t breath_t0=0; int breath_count=0;
inline void pump_duty(uint8_t d){ ledcWrite(PWM_CH,d); }
inline void pump_off(){ ledcWrite(PWM_CH,0); }
uint32_t mute_until_ms=0;
const uint32_t BREATH_TOTAL_MS=(uint32_t)BREATH_CYCLES*BREATH_PERIOD_MS;
inline bool detector_muted(){ return (pstate!=P_IDLE)||(millis()<mute_until_ms); }
inline void start_breath(){
  pstate=P_BREATH; breath_t0=millis(); breath_count=0;
  LED_G=true; LED_R=LED_Y=false; leds_apply();
  mute_until_ms=breath_t0+BREATH_TOTAL_MS+1000;
  Serial.println("pump,breath_start");
}

// ===== CONVERSIONE IDENTICA AL LOGGER =====
float adc_to_us(int adc){
  if(adc>=SERIAL_CALIBRATION) return NAN;
  long num=(4096L+2L*adc)*10000L;
  long ohm=num/(SERIAL_CALIBRATION-adc);
  if(ohm<=0) return NAN;
  return (1.0f/(float)ohm)*1e6f;
}

// ===== FEATURE 0–3 s + CLASSIFY =====
bool compute_features_and_classify(){
  int start=onset_pos_in_buf, end3=start+POST3_SAMPLES;
  if(end3>buf_len||start<0) return false;

  // baseline
  int npre=(start<PRE2_SAMPLES)?start:PRE2_SAMPLES;
  float base=0; if(npre>0){ for(int i=0;i<npre;i++) base+=scr_buf[start-1-i]; base/=npre; }

  // ampiezza e indice picco
  float pamp=0; int pidx=start;
  for(int k=start;k<end3;k++){ float y=scr_buf[k]-base; if(y<0)y=0; if(y>pamp){pamp=y;pidx=k;} }
  float t_peak_s=(pidx-start)/FS;

  // slope max
  float slope_max=0;
  for(int k=start+1;k<end3;k++){
    float y1=scr_buf[k]-base; if(y1<0)y1=0;
    float y0=scr_buf[k-1]-base; if(y0<0)y0=0;
    float sd=(y1-y0)*FS; if(sd>slope_max)slope_max=sd;
  }

  // AUC 0–3 s
  float auc=0;
  for(int k=start;k<end3-1;k++){
    float a=scr_buf[k]-base; if(a<0)a=0;
    float b=scr_buf[k+1]-base; if(b<0)b=0;
    auc+=0.5f*(a+b)*DT;
  }

  // shape: rise10-90 e width_half
  float y10=0.1f*pamp, y90=0.9f*pamp, half=0.5f*pamp;
  int k10=-1,k90=-1,kl=start,kr=end3-1;
  for(int k=start;k<=pidx;k++){ float y=scr_buf[k]-base; if(y<0)y=0; if(k10<0&&y>=y10)k10=k; if(k90<0&&y>=y90)k90=k; }
  for(int k=pidx;k>=start;k--){ float y=scr_buf[k]-base; if(y<0)y=0; if(y<=half){kl=k;break;} }
  for(int k=pidx;k<end3;k++){ float y=scr_buf[k]-base; if(y<0)y=0; if(y<=half){kr=k;break;} }
  float rise10_90=(k10>=0&&k90>k10)?(k90-k10)/FS:NAN;
  float width_half=(kr>kl)?(kr-kl)/FS:NAN;

  // energia / tonic / slope@1s
  float energy=0,mean=0; int n03=end3-start;
  for(int k=start;k<end3;k++){ float y=scr_buf[k]-base; if(y<0)y=0; energy+=y*y*DT; mean+=y; }
  mean/=fmaxf(1,n03);
  float var=0; for(int k=start;k<end3;k++){ float y=scr_buf[k]-base; if(y<0)y=0; float e=y-mean; var+=e*e; }
  var/=fmaxf(1,(n03-1)); float std03=sqrtf(fmaxf(var,0));
  int k1s=start+int(FS);
  float slope1s=(k1s+1<end3)?((fmaxf(scr_buf[k1s+1]-base,0)-fmaxf(scr_buf[k1s]-base,0))*FS):NAN;
  float tonic=0; if(npre>0){ for(int i=0;i<npre;i++) tonic+=scl_buf[start-1-i]; tonic/=npre; } else tonic=scl_buf[start];

// ===== FILTRO ANTI-ARTEFATTI MIGLIORATO =====
// Rifiuta eventi chiaramente rapidi/spigolosi ma lascia passare carezze lente
if (ENABLE_ARTIFACT_FILTER) {

  bool reject = false;

  // 1) Spike precoci e ripidi → tipico di tosse o urto
  if (t_peak_s < 0.35f && slope_max > 3.5f) reject = true;

  // 2) Impulso troppo breve e con poca area → micro-movimento
  if (width_half < 0.35f && auc < 0.05f) reject = true;

  // 3) AUC troppo piccola e picco basso → rumore o fluttuazione casuale
  if (auc < 0.03f && pamp < 0.25f) reject = true;

  // 4) Salita <0.2 s e pendenza >3 → spike netto (es. contrazione muscolare)
  if (rise10_90 < 0.20f && slope_max > 3.0f) reject = true;

  if (reject) {
    Serial.println("event,reject_artifact");
    LED_Y = LED_R = LED_G = false;
    leds_apply();
    return false;
  }
}


  // features → RF
  float feat[11]={pamp,t_peak_s,slope_max,auc,rise10_90,width_half,energy,mean,std03,slope1s,tonic};
  int y=clf.predict(feat);
  Serial.print("event,early_pred,"); Serial.println(y);

  // LED
  LED_Y=false;
  if(y==0){ LED_R=true; LED_G=false; leds_apply(); }
  if(y==1 && pstate==P_IDLE){ LED_R=false; LED_G=true; leds_apply(); start_breath(); }
  return true;
}

// ===== RUNTIME =====
float scl_prev=NAN, prev_scr=NAN; unsigned long last_ms=0;

void setup(){
  Serial.begin(115200); delay(100);
  lp.set(0.06745527f,0.13491055f,0.06745527f,-1.14298050f,0.41280160f);
  ledcSetup(PWM_CH,PWM_FREQ,PWM_RES); ledcAttachPin(PIN_PUMP,PWM_CH); pump_off();
  pinMode(YELLOW_LED,OUTPUT); pinMode(RED_LED,OUTPUT); pinMode(GREEN_LED,OUTPUT);
  Serial.println("ready");
}

void loop(){
  if(millis()-last_ms<50){ pump_fsm(); return; }
  last_ms=millis();

  int adc=analogRead(GSR_PIN);
  float us=adc_to_us(adc);

  // Plot grezzo µS (formato CSV semplice)
  if (ENABLE_PLOT) {
    Serial.print(0); Serial.print(',');
    Serial.print(isnan(us)?0.0f:us); Serial.print(',');
    Serial.println(40);
  }

  if(isnan(us)){ pump_fsm(); return; }

  float us_lp=lp.step(us);
  if(isnan(scl_prev)) scl_prev=us_lp;
  float scl=ALPHA_SCL*us_lp+(1.0f-ALPHA_SCL)*scl_prev; scl_prev=scl;
  float scr=us_lp-scl;
  float d=(isnan(prev_scr)?0:(scr-prev_scr)*FS); prev_scr=scr;
  sigma_push(d); float sigma_d=rolling_sigma();

  if(buf_len==MAX_BUF){
    for(int i=1;i<buf_len;i++){scr_buf[i-1]=scr_buf[i];scl_buf[i-1]=scl_buf[i];}
    buf_len--; if(onset_pos_in_buf>=0) onset_pos_in_buf--;
  }
  scr_buf[buf_len]=scr; scl_buf[buf_len]=scl; buf_len++;

  if(detector_muted()){ pump_fsm(); return; }

  switch(state){
    case IDLE:
      if(d>K_SIGMA*sigma_d){
        if(buf_len>POST3_SAMPLES){
          float grow=scr_buf[buf_len-1]-scr_buf[max(0,buf_len-1-POST3_SAMPLES)];
          if(grow>A_MIN_US){
            onset_pos_in_buf=buf_len-1; state=WAIT_POST;
            Serial.println("event,onset");
            LED_Y=true; LED_R=LED_G=false; leds_apply();
          }
        }
      }
      break;

    case WAIT_POST:
      if((buf_len-1-onset_pos_in_buf)>=POST3_SAMPLES){
        compute_features_and_classify();
        refract_left=int(REFRACT_S*FS); state=REFRACT;
      }
      break;

    case REFRACT:
      if(--refract_left<=0){
        state=IDLE; onset_pos_in_buf=-1;
        Serial.println("event,ready");
        LED_Y=LED_R=LED_G=false; leds_apply();
      }
      break;
  }
  pump_fsm();
}

// ===== FSM POMPA: respiro PWM =====
void pump_fsm(){
  switch(pstate){
    case P_IDLE: break;
    case P_BREATH:{
      uint32_t t=millis()-breath_t0;
      float phase=(float)(t%BREATH_PERIOD_MS)/(float)BREATH_PERIOD_MS;
      float env=0.5f*(1.0f-cosf(2.0f*3.1415926f*phase));
      uint8_t duty=(uint8_t)(DUTY_MIN+env*(DUTY_MAX-DUTY_MIN));
      pump_duty(duty);
      if(t>=(uint32_t)((breath_count+1)*BREATH_PERIOD_MS)){
        breath_count++;
        if(breath_count>=BREATH_CYCLES){
          pump_off(); pstate=P_IDLE; LED_G=false; leds_apply();
          Serial.println("pump,breath_end");
        }
      }
    } break;
  }
}

