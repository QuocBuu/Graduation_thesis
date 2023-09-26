
// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; //Paramaters
    float eprev, eintegral; //Storage

  public:
  //Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  //A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
  //error
  int e = target - value;

  //derivative
  float dedt = (e - eprev)/(deltaT);

  //integral
  eintegral = eintegral + e*deltaT;

  //control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  //motor power
  float pwr = fabs(u);
  if(pwr > umax){
    pwr = umax
  }
  }
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
