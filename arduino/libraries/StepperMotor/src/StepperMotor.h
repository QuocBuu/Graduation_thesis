#ifndef StepperMotor_h
#define StepperMotor_h

class StepperMotor {
    private:
        int pin_pul;
        int pin_dir;
        bool direction;
        bool startDirection;
        double stepsPerUnit;
        long positon;
        double last_step_time;
        double step_delay;

    public:
        StepperMotor(int p, int d);
        void setZero();
        void setSpeed(double whatSpeed);
        void setStepsPerUnit(double ppu);
        void setStartDirection(bool dir);
        void moveTo(double absolute);
        double currentPosition();        
};

#endif