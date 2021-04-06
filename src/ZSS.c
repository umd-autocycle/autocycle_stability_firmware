#define RETRACT_TIME 2.0f
class ZSS{
    void zssDeploy(){
        digitalWrite(retractPin,LOW);
        digitalWrite(deployPin,HIGH); //deployPin connects to the relay that controls positive voltage to the actuator
    }
    void zssRetract(){
        digitalWrite(deployPin,LOW);
        digitalWrite(retractPin,HIGH); //retractPin connects to the relay that controls negative voltage to the actuator
    }
    void zssFreeze(){
        digitalWrite(retractPin,LOW);
        digitalWrite(deployPin,LOW);
    }
}
