class TwoMotorDrive extends DriveBase
{
    public TwoMotorDrive()
    {
        numMotors = 2;
        ArrayList<DcMotor> motors = new ArrayList<>(numMotors);
    }

    public void init(HardwareMap hardwareMap)
    {
        motors[LEFT] = hardwareMap.get(DcMotor.class, "left_motor");
        motors[RIGHT] = hardwareMap.get(DcMotor.class, "right_motor");
        isInitalized = true;

        motors[LEFT].setDirection(DcMotor.Direction.REVERSE);
        
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(double drive, double strafe, double twist)
    {

    }
}