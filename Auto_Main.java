package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Auto_Main", group="Auto")
public class Auto_Main extends LinearOpMode {

    /* Declare OpMode members. */
    
    private static final int kNOT_SET = -1;
    
    private static final int kSTART_LOCATION_1 = 0;
    private static final int kSTART_LOCATION_2 = 1;
    private static final int kSTART_LOCATION_3 = 2;
    public int current_start_location = kNOT_SET;
    
    private static final int kEND_LOCATION_NET_ZONE = 0;
    private static final int kEND_LOCATION_ASCENT_ZONE = 1;
    private static final int kBLUE = 0;
    private static final int kRED = 1;
    public int alliance_color = kNOT_SET;
    
    public DcMotor      left_front_drive  = null;
    public DcMotor      left_rear_drive   = null;
    public DcMotor      right_front_drive = null;
    public DcMotor      right_rear_drive  = null;
    
    public DcMotor      arm_hang_motor = null;
    public DcMotor      arm_extend_motor = null;
    public DcMotor      shoulder_motor = null;
    public IMU imu;
    public DigitalChannel led_red = null;
    public DigitalChannel led_green = null;
    public Servo      gripper_servo = null;
    public CRServo        wrist_servo = null;
    public DistanceSensor distance_sensor = null;

    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    public double current_speed = MAX_MOVE_SPEED;
    
    
    public static final double GRIPPER_OPEN = 0.75;
    public static final double GRIPPER_CLOSED = 1.0;
    public static final int GRIPPER_STATUS_OPEN = 0;
    public static final int GRIPPER_STATUS_CLOSED = 1;
    public int current_gripper_status = GRIPPER_STATUS_CLOSED;
    
    public static final int ARM_STATUS_CLOSED = 0;
    public static final int ARM_STATUS_PICKUP = 1;
    public static final int ARM_STATUS_LOW_BASKET = 2;
    public static final int ARM_STATUS_HIGH_BASKET = 3;
    public static final int ARM_STATUS_HIGH_CHAMBER = 4;
    public int current_arm_status = ARM_STATUS_CLOSED;
    public String current_arm_status_name = "ARM_STATUS_CLOSED"; 
   

    public static final int HIGH_BASKET_LOCATION= 4500;//4700
    public static final int LOW_BASKET_LOCATION= 2500;
    public static final int PICKUP_LOCATION= 1300;
    public static final int HIGH_CHAMBER_LOCATION = 1750;

    public int counter = 0;
    public Gamepad saved_gamepad1 = new Gamepad();
    public String alliance = "blue"; 
    public String audience = "none";
    public String backside = "none";
    public String position = "none";
    public double motors_count_per_revolution = 28;
    public double gear_ratio = 3.61*5.23;
    // we have a four to one and five to one gear ratio//
    public double cpr = gear_ratio*motors_count_per_revolution;
    public double diamiter = 104; // millimeteres//
    public double curcumfrence = Math.PI*diamiter;

    private ElapsedTime runtime = new ElapsedTime();
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection  IMU_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_FACING_DIRECTION);

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 3.61*5.23;
    static final double     WHEEL_DIAMETER_INCHES   = 104/25.4;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public  double          headingError  = 0;
    public  double          targetHeading = 0;
    private double          driveSpeed    = 0;
    private double          turnSpeed     = 0;
    
    @Override
    public void runOpMode() {

        init_all();
        GripperClosed();
        wrist_servo.setPower(-1);
        // Wait for the game to start (driver presses START)
        while(!this.isStarted() && !this.isStopRequested()) {
            if (gamepad1.x || gamepad2.x) {
                alliance_color = kBLUE;
            }
            if (gamepad1.b || gamepad2.b) {
                alliance_color = kRED;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                current_start_location = kSTART_LOCATION_1;// onedunkandpark 
            }
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                current_start_location = kSTART_LOCATION_2;//twodunkandpark
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                current_start_location = kSTART_LOCATION_3;//specimen (food)
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                current_start_location = kNOT_SET;
            }
            telemetryChoice();
            telemetry.addData("Current Shoulder", shoulder_motor.getCurrentPosition());
            telemetry.update();
        }
        //waitForStart();
        imu.resetYaw();
        reset_shoulder();
        if (current_start_location == kSTART_LOCATION_1)
        {
            OneDunkAndPark(ARM_STATUS_HIGH_BASKET);
        }
        else if (current_start_location == kSTART_LOCATION_2)
        {
            TwoDunkAndPark(ARM_STATUS_HIGH_BASKET);
        }
        else if (current_start_location == kSTART_LOCATION_3)
        {
            Food();
        }
       else 
       {
           
       }
    }

    public void TwoDunkAndPark(int basket)
    {
        MoveLeft(DRIVE_SPEED, 10, 5);
        //MoveShoulder(DRIVE_SPEED, 1300, 5);
        
        ShoulderUp();
        SetArmPosition(basket);
        sleep(500);
        MoveBackward(DRIVE_SPEED, 4, 5);
        GripperOpen();
        sleep(500);
        MoveForward(DRIVE_SPEED, 4, 5);
        SetArmPosition(ARM_STATUS_CLOSED);
        //MoveShoulder(DRIVE_SPEED, 0, 5);
        
        ShoulderDown();
        rotateBy(45); // rotate left 45 degrees
        MoveRight(DRIVE_SPEED, 4, 5);
        GripperOpen();
        ExtendArm(1, 2768, 5);
        wrist_servo.setPower(1);
        sleep(500);
        GripperClosed();
        SetArmPosition(ARM_STATUS_CLOSED);
        
        rotateBy(-45);
        ShoulderUp();
        SetArmPosition(basket);
        MoveBackward(DRIVE_SPEED, 6, 5);
        wrist_servo.setPower(-1);
        sleep(1000);
        GripperOpen();
        sleep(500);
        wrist_servo.setPower(1);
        sleep(500);
        MoveForward(DRIVE_SPEED, 4, 5);
        rotateBy(20);
        MoveForward(1, 42, 5);
        SetArmPosition(ARM_STATUS_CLOSED);
        ShoulderDown();
    }
    
    public void Food()
    {
        //hook starting specimen
        MoveBackward(DRIVE_SPEED, 21, 5);
        ShoulderUp();
        ExtendArm(1, 500, 5 );
        MoveBackward(DRIVE_SPEED, 2.5, 5);
        ExtendArm(1, 2400, 5 );
        GripperOpen();
        SetArmPosition(ARM_STATUS_CLOSED);
        MoveForward(DRIVE_SPEED, 5, 5);
        ShoulderDown();

        //first mark
        MoveLeft(DRIVE_SPEED, 38, 5);// used to be 48 not 42
        MoveBackward(DRIVE_SPEED, 32, 5);
        MoveLeft(DRIVE_SPEED, 10, 5);
        MoveForward(DRIVE_SPEED, 45, 5);
        double current = getHeading();
        rotateBy(-current);
        
        //second specimen
        MoveBackward(DRIVE_SPEED, 7.5, 5);
        ShoulderDown();
        wrist_servo.setPower(-0.3);
        GripperOpen();
        double current_distance = distance_sensor.getDistance(DistanceUnit.INCH);
        MoveForward(.3, current_distance - 4, 5);
        //MoveForward(DRIVE_SPEED, 6, 5);
        //MoveLeft(DRIVE_SPEED, 4, 5);
        //GripperOpen();
        //MoveForward(DRIVE_SPEED, 5, 5);
        //wrist_servo.setPower(1);
        sleep(1000);
        GripperClosed();
        ShoulderUp();
        
        //Hooking it on the bar
        MoveRight(DRIVE_SPEED, 38, 5);
        sleep(500);
        MoveBackward(DRIVE_SPEED, 21, 5 );
        ExtendArm(1, 2400, 5 );
        
        sleep(1000);
        /*(DRIVE_SPEED, 12, 5);
        wrist_servo.setPower(-1);
        GripperClosed();
        ShoulderUp();
        MoveRight(DRIVE_SPEED, 48, 5);
        */
        
        //second mark
        //MoveBackward(DRIVE_SPEED, 48, 5);
        //MoveLeft(DRIVE_SPEED, 9, 5);
        //MoveForward(DRIVE_SPEED, 45, 5);
    }
    public void Food2() //org
    {
        MoveBackward(DRIVE_SPEED, 20.5, 5);
        ShoulderUp();
        SetArmPosition(ARM_STATUS_HIGH_CHAMBER);
        MoveBackward(DRIVE_SPEED, 2, 5);
        //GripperOpen();
        ExtendArm(.5, HIGH_CHAMBER_LOCATION- 600, 5 );
        GripperOpen();
        SetArmPosition(ARM_STATUS_CLOSED);
        MoveForward(DRIVE_SPEED, 22, 5);
        MoveLeft(DRIVE_SPEED, 38, 5);// used to be 48 not 42
        MoveBackward(DRIVE_SPEED, 48, 5);
        MoveLeft(DRIVE_SPEED, 8, 5);
        MoveForward(DRIVE_SPEED, 48, 5);
    }
    
    public void OneDunkAndPark(int basket)
    {
        MoveLeft(DRIVE_SPEED, 10, 5);
        //MoveShoulder(DRIVE_SPEED, 1300, 5);
        
        ShoulderUp();
        SetArmPosition(basket);
        MoveBackward(DRIVE_SPEED, 5, 5);
        GripperOpen();
        sleep(500);
        MoveForward(DRIVE_SPEED, 5, 5);
        SetArmPosition(ARM_STATUS_CLOSED);
        //MoveShoulder(DRIVE_SPEED, 0, 5);
        
        ShoulderDown();
        //rotateBy(15); // rotate left 45 degrees
        MoveForward(DRIVE_SPEED, 24, 5);
        rotateBy(35);
        MoveForward(DRIVE_SPEED, 10, 5);
        /*
        SetArmPosition(ARM_STATUS_PICKUP);
        MoveForward(DRIVE_SPEED, 10, 5);
        wrist_servo.setPower(1);
        GripperClosed();
        */
        
        
    }
    public void ShoulderUp()
    {
        shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor.setPower(.75);
        sleep(1000);
        shoulder_motor.setPower(0);
    }
    public void ShoulderDown()
    {
        shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor.setPower(-.5);
        sleep(1000);
        shoulder_motor.setPower(0);
    }
    public void MoveForward(double speed, double inches, double timeoutS)
    {
        encoderDrive(speed, inches, inches, inches, inches, timeoutS);
    }
    public void MoveBackward(double speed, double inches, double timeoutS)
    {
        encoderDrive(speed, -inches, -inches, -inches, -inches, timeoutS);
    }
    public void MoveRight(double speed, double inches, double timeoutS)
    {
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }
    public void MoveLeft(double speed, double inches, double timeoutS)
    {
        encoderDrive(speed, -inches, inches, inches, -inches, timeoutS);
    }
    private void reset_shoulder()
    {
        shoulder_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double left_front_driveInches, double right_front_driveInches, 
                             double left_rear_driveInches, double right_rear_driveInches,
                             double timeoutS) {
        int newleft_front_driveTarget;
        int newright_front_driveTarget;
        int newleft_rear_driveTarget;
        int newright_rear_driveTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newleft_front_driveTarget = left_front_drive.getCurrentPosition() + (int)(left_front_driveInches * COUNTS_PER_INCH);
            newright_front_driveTarget = right_front_drive.getCurrentPosition() + (int)(right_front_driveInches * COUNTS_PER_INCH);
            newleft_rear_driveTarget = left_rear_drive.getCurrentPosition() + (int)(left_rear_driveInches * COUNTS_PER_INCH);
            newright_rear_driveTarget = right_rear_drive.getCurrentPosition() + (int)(right_rear_driveInches * COUNTS_PER_INCH);
            
            left_front_drive.setTargetPosition(newleft_front_driveTarget);
            right_front_drive.setTargetPosition(newright_front_driveTarget);
            left_rear_drive.setTargetPosition(newleft_rear_driveTarget);
            right_rear_drive.setTargetPosition(newright_rear_driveTarget);

            // Turn On RUN_TO_POSITION
            left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_rear_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_rear_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left_front_drive.setPower(Math.abs(speed));
            right_front_drive.setPower(Math.abs(speed));
            left_rear_drive.setPower(Math.abs(speed));
            right_rear_drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (left_front_drive.isBusy() && right_front_drive.isBusy() && 
                   left_rear_drive.isBusy() && right_rear_drive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  "Front %7d :%7d Rear %7d :%7d", newleft_front_driveTarget,  
                newright_front_driveTarget,  newleft_rear_driveTarget,  newright_rear_driveTarget);
                telemetry.addData("Currently at",  " Front %7d :%7d Rear %7d :%7d", left_front_drive.getCurrentPosition(), right_front_drive.getCurrentPosition(), left_rear_drive.getCurrentPosition(), right_rear_drive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            left_front_drive.setPower(0);
            right_front_drive.setPower(0);
            left_rear_drive.setPower(0);
            right_rear_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public boolean isBusy()
    {
        return (left_front_drive.isBusy() && right_front_drive.isBusy() && left_rear_drive.isBusy() && right_rear_drive.isBusy());
    }
    
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    
    
    public void init_all() {
        // Define and Initialize Motors
        init_chassis();
        init_arm_extend();
        init_arm_hang();
        init_shoulder();
        init_gripper();
        init_wrist();
        
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
    }

    private void init_gripper()
    {
        gripper_servo = hardwareMap.get(Servo.class, "gripper_servo");
        //gripper_servo.resetDeviceConfigurationForOpMode();
    }
    
    private void init_wrist()
    {
        wrist_servo = hardwareMap.get(CRServo.class, "wrist_servo");
        //wrist_servo.resetDeviceConfigurationForOpMode();
    }
    
    private void init_leds()
    {
        led_red = hardwareMap.get(DigitalChannel.class, "red");
        led_green = hardwareMap.get(DigitalChannel.class, "green");    
        
        //change LED mode from input to output
        led_red.setMode(DigitalChannel.Mode.OUTPUT);
        led_green.setMode(DigitalChannel.Mode.OUTPUT);
    }
    
    private void init_chassis()
    {
        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_front_drive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void init_arm_extend()
    {
        arm_extend_motor = hardwareMap.get(DcMotor.class, "arm_extend_motor");
        arm_extend_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_extend_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_extend_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void init_arm_hang()
    {
        arm_hang_motor = hardwareMap.get(DcMotor.class, "arm_hang_motor");
        arm_hang_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void init_shoulder()
    {
        shoulder_motor = hardwareMap.get(DcMotor.class, "shoulder_motor");
        shoulder_motor.setDirection(DcMotor.Direction.FORWARD);
        shoulder_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
    public void move(double left_front, double right_front, double left_back, double right_back)
    {
        left_front_drive.setPower(left_front);
        left_rear_drive.setPower(left_back);
        
        right_front_drive.setPower(right_front);
        right_rear_drive.setPower(right_back);  
    }
    
    public void MoveShoulder(double speed, double target_pos, double timeoutS)
    {
        // Ensure that the OpMode is still active

            // Determine new target position, and pass to motor controller
            //new_distance = arm_extend_motor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            
            int current = shoulder_motor.getCurrentPosition();
            //speed = Math.abs(speed);
        
            target_pos = current + target_pos;
            shoulder_motor.setTargetPosition((int)target_pos);
            
            // Turn On RUN_TO_POSITION
            shoulder_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            shoulder_motor.setPower(speed);

            while ((runtime.seconds() < timeoutS) 
                    //(abs(target_pos - new_distance) > 100) &&
                   && shoulder_motor.isBusy()
                   ) 
            {
                current = shoulder_motor.getCurrentPosition();

                // Display it for the driver.
                telemetry.addData("Running to",  target_pos);
                telemetry.addData("Currently at", current);
                telemetry.update();
            }

            // Stop all motion;
            shoulder_motor.setPower(0);
            
            // Turn off RUN_TO_POSITION
            shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void ExtendArm(double speed, double pos, double timeoutS)
    {
        // Ensure that the OpMode is still active
            // Determine new target position, and pass to motor controller
            //new_distance = arm_extend_motor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            
            int new_distance = arm_extend_motor.getCurrentPosition();
            speed = Math.abs(speed);
        
            arm_extend_motor.setTargetPosition((int)pos);
            
            // Turn On RUN_TO_POSITION
            arm_extend_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            arm_extend_motor.setPower(speed);

            while ((runtime.seconds() < timeoutS) &&
                   arm_extend_motor.isBusy()) 
            {
                // Display it for the driver.
                telemetry.addData("Running to",  new_distance);
                telemetry.addData("Currently at", arm_extend_motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            arm_extend_motor.setPower(0);
            
            // Turn off RUN_TO_POSITION
            arm_extend_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void GripperOpen()
    {
         gripper_servo.setPosition(GRIPPER_OPEN);
         current_gripper_status = GRIPPER_STATUS_OPEN;
    }
    public void GripperClosed()
    {
         gripper_servo.setPosition(GRIPPER_CLOSED);
         current_gripper_status = GRIPPER_STATUS_CLOSED;
    }
    public void SetArmPosition(int pos)
    {
        switch(pos)
        {
            case ARM_STATUS_CLOSED:
                {
                    ExtendArm(1.0, 0,3);
                    current_arm_status = ARM_STATUS_CLOSED;
                    current_arm_status_name = "ARM_STATUS_CLOSED";
                    break;
                }
                 case ARM_STATUS_PICKUP:
                {
                    ExtendArm(1.0, PICKUP_LOCATION, 3);
                    current_arm_status = ARM_STATUS_PICKUP ;
                     current_arm_status_name = "ARM_STATUS_PICKUP";
                    break;
                }
                 case ARM_STATUS_LOW_BASKET:
                {
                    ExtendArm(1.0, LOW_BASKET_LOCATION, 4);
                    current_arm_status = ARM_STATUS_LOW_BASKET;
                    current_arm_status_name = "ARM_STATUS_LOW_BASKET";
                    break;
                }
                 case ARM_STATUS_HIGH_BASKET:
                {
                    ExtendArm(1.0, HIGH_BASKET_LOCATION, 5);
                    current_arm_status = ARM_STATUS_HIGH_BASKET;
                    current_arm_status_name = "ARM_STATUS_HIGH_BASKET";
                    break;
                }
                case ARM_STATUS_HIGH_CHAMBER:
                {
                    ExtendArm(1.0, HIGH_CHAMBER_LOCATION, 7);
                    current_arm_status = ARM_STATUS_HIGH_CHAMBER;
                    current_arm_status_name = "ARM_STATUS_HIGH_CHAMBER";
                    break;
                }
            
        }
    }
    
    public void rotate(double degrees)
    {
    //positive left
        //All angles are in the range of -180 degrees to 180 degrees.
        double heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double normalized = heading_deg % 360;
        
        double diff = (degrees - heading_deg + 540) % 360 - 180;
        
        double speed=.2; //positive speed, turn left
                        //negative speed, turn right
        if (diff < 0)
            speed *= -1;
            
        left_front_drive.setPower(speed);
        left_rear_drive.setPower(speed);
        right_front_drive.setPower(-speed);
        right_rear_drive.setPower(-speed);
        
        boolean finished = false;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && !finished && (runtime.seconds() < 10))
        {
            heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            diff = (degrees - heading_deg + 540) % 360 - 180;
            telemetry.addData("Running to (raw)",  degrees);
            telemetry.addData("Running to (normalized)",  normalized);
            telemetry.addData("Currently at", heading_deg);
            telemetry.addData("diff", diff);
            telemetry.update();
  
            if (Math.abs(heading_deg - degrees) < 1)
            {
                finished = true;
            }
        }
        move(0,0,0,0);
        //shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void rotateBy(double degrees)
    {
        //set our current heading to 0
        imu.resetYaw();
        
        //positive left
        //All angles are in the range of -180 degrees to 180 degrees.
        double heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double normalized = heading_deg % 360;
        double diff = (degrees - heading_deg + 540) % 360 - 180;
        
        double speed=.2; //positive speed, turn left
                        //negative speed, turn right
        if (degrees < 0)
            speed *= -1;
            
        left_front_drive.setPower(-speed);
        left_rear_drive.setPower(-speed);
        right_front_drive.setPower(speed);
        right_rear_drive.setPower(speed);
        
        boolean finished = false;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive() && !finished && (runtime.seconds() < 10))
        {
            heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //diff = (degrees - heading_deg + 540) % 360 - 180;
            telemetry.addData("Running to",  degrees);
            //telemetry.addData("Running to (normalized)",  normalized);
            telemetry.addData("Current at", heading_deg);
            //telemetry.addData("diff", diff);
            telemetry.update();
  
            if (Math.abs(heading_deg - degrees) < 1)
            {
                finished = true;
            }
        }
        move(0,0,0,0);
        //shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    void telemetryChoice()
    {
        if (alliance_color == kBLUE)
        {
            telemetry.addData("Current Alliance", "Blue");
        }
        else if (alliance_color == kRED)
        {
            telemetry.addData("Current Alliance", "Red");
        }
        else
        {
            telemetry.addData("Current Alliance", "None");            
        }
        if (current_start_location == kSTART_LOCATION_1 )
        {
            telemetry.addData("Start Position   ", "One dunk and park");
        }
        else if (current_start_location == kSTART_LOCATION_2)
        {
            telemetry.addData("Start Position   ", "Two dunk and park");
        }
        else if (current_start_location == kSTART_LOCATION_3)
        {
            telemetry.addData("Start Position   ", "Specimen (Food)");
        }
        else
        {
            telemetry.addData("Start Position   ", "None");
        }  
    }
}
