
#include<Servo.h>


class _servo{
  private:
  double st;
  double end;
  double time;
  double vel;
  double ts;

  public:
  _servo(){}

  _servo( double st,double end,double time){
    this->st=st;
    this->end=end;
    this->time=time;
    this->vel=(end-st)/time;
    this->ts=millis();

  }

  double move(){
    double t=(millis()-this->ts)/1000;

    if(t>=this->time){
      return this->end;
    }
    return this->st+this->vel*t;
  

  }

  void reset(double st,double end,double time){
    this->st=st;
    this->end=end;
    this->time=time;
    this->vel=(end-st)/time;
    this->ts=millis();

  }




};

_servo s(0,180,10);
_servo s2(180,0,10);
Servo a;
int b=0;
int angle=0;

void setup() {
  a.attach(5);

}

void loop() {
  if(b==0){
    angle=s.move();
    if(angle>=180){
      b=1;
      s.reset(0,180,10);
    }
    
  }
  else{
    if(angle<=0){
      b=0;
      s2.reset(180,0,10);
    }
    angle=s2.move();
  }
  a.write(angle);

 

}
