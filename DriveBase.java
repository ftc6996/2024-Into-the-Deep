abstract class DriveBase
{
    public enum MotorPosition {
        LEFT_FRONT = 0,
        RIGHT_FRONT,
        LEFT_REAR, 
        RIGHT_REAR,
        LEFT = LEFT_FRONT,
        RIGHT = RIGHT_FRONT
    }

    abstract public void init(HardwareMap hardwareMap);
    abstract public void move(double drive, double strafe, double twist);

    public void setMode(DcMotor.RunMode mode)
    {
        if (!isInitalized)
            return;

        for (DcMotor motor : motors) {
            motor.setMode(mode);
        } 
    }

    public void setDirection(MotorPosition motor, DcMotor.Direction mode)
    {
        if (motor < motors.size())
        {
            motors[motor].setDirection(mode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode)
    {
        if (!isInitalized)
            return;

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }      
    }

    public void stop()
    {
        if (!isInitalized)
            return;

        for (DcMotor motor : motors) {
            motor.setPower(0);
        } 
    }

    protected boolean isInitalized = false;
    protected final int numMotors;
    protected DcMotor [] motors;
}