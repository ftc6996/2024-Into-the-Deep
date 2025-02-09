package org.firstinspires.ftc.teamcode;
//gamepad1.back    == robot vs field centric
//gamepad1.options == reset heading
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverController", group="TeleOp")
public class DriverController extends OpMode{

    static final double COUNTS_PER_INCH = 1.0;
    /* Declare OpMode members. */
    public DcMotor      left_front_drive  = null;
    public DcMotor      left_rear_drive   = null;
    public DcMotor      right_front_drive = null;
    public DcMotor      right_rear_drive  = null;
    public DcMotor      intake_motor = null;
    public DcMotor      arm_hang_motor = null;
    public DcMotor      arm_extend_motor = null;
    public DcMotor      shoulder_motor = null;
    public IMU imu;
    public DigitalChannel led_red = null;
    public DigitalChannel led_green = null;
    public Servo      gripper_servo = null;
    public CRServo        wrist_servo = null;
    public TouchSensor shoulder_sensor = null;
    public TouchSensor arm_sensor = null;
    
    static final double INTAKE_SPEED = 1.0;
    static final double OUTTAKE_SPEED = -1.0;
    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    public double current_speed = .5;
    public double current_shoulder_speed = .7;
    
    public static final double GRIPPER_OPEN = 0.70;
    public static final double GRIPPER_CLOSED = 1.0;
    public static final int GRIPPER_STATUS_OPEN = 0;
    public static final int GRIPPER_STATUS_CLOSED = 1;
    public int current_gripper_status = GRIPPER_STATUS_CLOSED;

    public boolean ARM_IS_MOVING = false;    
    public static final int ARM_STATUS_CLOSED = 0;
    public static final int ARM_STATUS_PICKUP = 1;
    public static final int ARM_STATUS_LOW_BASKET = 2;
    public static final int ARM_STATUS_HIGH_BASKET = 3;
    public int current_arm_status = ARM_STATUS_CLOSED;
    public String current_arm_status_name = "ARM_STATUS_CLOSED"; 
    public static final int ARM_INVALID_POS = -6996;
    public int current_target_position = ARM_INVALID_POS;
    public boolean override_checking = false;
    
    public static final int HIGH_BASKET_LOCATION= 4600;//used to be 4700
    public static final int LOW_BASKET_LOCATION= 2500;
    public static final int PICKUP_LOCATION= 1800;
    
    public static final int ARM_EXTEND_MAX= 2050;
    public static final int ARM_HANG_MAX = 12180; //1197 if needed
    
    public static final int SHOULDER_LOW_POSITION = 600;
    
    static final int    DRIVER_MODE_FIELD = 0;
    static final int    DRIVER_MODE_ROBOT = 1;
    public int driver_mode = DRIVER_MODE_ROBOT;
    
    
    
    static final int    LED_STATUS_OFF = 0;
    static final int    LED_STATUS_GREEN = 1;
    static final int    LED_STATUS_RED = 2;
    static final int    LED_STATUS_YELLOW = 3;
    public int current_led_status = LED_STATUS_OFF;
    public int counter = 0;
    public Gamepad saved_gamepad1 = new Gamepad();
    public Gamepad saved_gamepad2 = new Gamepad();
    
    private ElapsedTime runtime = new ElapsedTime();
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection  IMU_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_FACING_DIRECTION);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        init_chassis();
        init_arm_extend();
        init_arm_hang();
        init_shoulder();
        init_intake();
        init_gripper();
        init_wrist();
        
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
    }

    private void init_gripper()
    {
        gripper_servo = hardwareMap.get(Servo.class, "gripper_servo");
        gripper_servo.setDirection(Servo.Direction.REVERSE);
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
        arm_extend_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_extend_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        arm_sensor = hardwareMap.get(TouchSensor.class, "arm_sensor");
    }
    private void reset_arm_extend()
    {
        arm_extend_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_extend_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void init_arm_hang()
    {
        arm_hang_motor = hardwareMap.get(DcMotor.class, "arm_hang_motor");
        arm_hang_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void reset_arm_hang()
    {
        arm_hang_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void init_shoulder()
    {
        shoulder_motor = hardwareMap.get(DcMotor.class, "shoulder_motor");
        shoulder_motor.setDirection(DcMotor.Direction.REVERSE);
        shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        shoulder_sensor = hardwareMap.get(TouchSensor.class, "shoulder_sensor");
    }
    private void reset_shoulder()
    {
        shoulder_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void init_intake()
    {
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        reset_arm_hang();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: 
        //  drive (front-and-back),
        //  strafe (left-and-right), and 
        //  twist (rotating the whole chassis).
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        double intake_trigger = gamepad2.left_trigger;   //up
        double outtake_trigger = gamepad2.right_trigger; //down

        double hang_left = gamepad1.left_trigger;
        double hang_right = gamepad1.right_trigger;
        
        boolean strafe_left = gamepad1.left_bumper;
        boolean strafe_right = gamepad1.right_bumper;
        
        double arm_extend_stick = -gamepad2.left_stick_y;
        double shoulder_stick = gamepad2.right_stick_y;
        
        if (Math.abs(shoulder_stick) < .15)
        {
            shoulder_stick = 0;
        }
        shoulder_stick = shoulder_stick * current_shoulder_speed;
        
        //driver controls
        if (gamepad1.options)
        {
            imu.resetYaw();
        }
        if (gamepad1.back && !saved_gamepad1.back)
        {
            if (driver_mode == DRIVER_MODE_ROBOT)
            {
                driver_mode = DRIVER_MODE_FIELD;
            }
            else
            {
                driver_mode = DRIVER_MODE_ROBOT;
            }
        }

        if (gamepad1.dpad_up && !saved_gamepad1.dpad_up)
        {
            current_speed += SPEED_INCREMENT;
            if (current_speed > MAX_MOVE_SPEED)
            {
                current_speed = MAX_MOVE_SPEED;
            }
        }
        else if (gamepad1.dpad_down && !saved_gamepad1.dpad_down)
        {
            current_speed -= SPEED_INCREMENT;
            if (current_speed < MIN_MOVE_SPEED)
            {
                current_speed = MIN_MOVE_SPEED;
            }
        }
        if (gamepad2.a && !saved_gamepad2.a)
        {
            if (current_gripper_status == GRIPPER_STATUS_OPEN)
            {
                //gripper_status = GRIPPER_STATUS_CLOSED;
                GripperClosed();
            }
            else
            {
                //gripper_status = GRIPPER_STATUS_OPEN;
                GripperOpen();
            }
        }
        
        if (gamepad2.dpad_up && !saved_gamepad2.dpad_up)
        {
            current_shoulder_speed += SPEED_INCREMENT;
            if (current_shoulder_speed > MAX_MOVE_SPEED)
            {
                current_shoulder_speed = MAX_MOVE_SPEED;
            }
        }
        else if (gamepad2.dpad_down && !saved_gamepad2.dpad_down)
        {
            current_shoulder_speed -= SPEED_INCREMENT;
            if (current_shoulder_speed < MIN_MOVE_SPEED)
            {
                current_shoulder_speed = MIN_MOVE_SPEED;
            }
        }


        if (ARM_IS_MOVING)
        {
            if (!arm_extend_motor.isBusy())
            {
                ARM_IS_MOVING = false;
                // Stop all motion;
                arm_extend_motor.setPower(0);
                // Turn off RUN_TO_POSITION
                arm_extend_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        if (gamepad2.b && !saved_gamepad2.b)
        {
            if (current_arm_status == ARM_STATUS_CLOSED)
            {
                SetArmPosition(ARM_STATUS_PICKUP);   
            }
            else
            {
                SetArmPosition(ARM_STATUS_CLOSED);   
            }
        }
        // pressing x Goes to the low basket
        if (gamepad2.x && !saved_gamepad2.x)
        {
            if (!IsShoulderLow())
            {
                SetArmPosition(ARM_STATUS_LOW_BASKET);
            }
        }
        // pressing y goees to the high basket
        if (gamepad2.y && !saved_gamepad2.y)
        {
            if (!IsShoulderLow())
            {
                SetArmPosition(ARM_STATUS_HIGH_BASKET); 
            }
        }
        
        
        //intake controls
        if ((intake_trigger > 0))
        {
            //wrist_servo.setPosition(1);
            wrist_servo.setPower(1);
        }
        else if (outtake_trigger > 0)
        {
            //wrist_servo.setPosition(0);
            wrist_servo.setPower(-.8);
        }
        else 
        {
            //intake_motor.setPower(0.0);
            wrist_servo.setPower(0);
        }
        
        //arm controls
        if (arm_extend_stick != 0)
        {
            //TODO: limit arm extending??
            if (IsShoulderLow())
            {
                if (arm_extend_motor.getCurrentPosition()< ARM_EXTEND_MAX)
                {
                     arm_extend_motor.setPower(arm_extend_stick); 
                } 
                else 
                {
                 arm_extend_motor.setPower(0); 

                }
            }
            else
            {
                arm_extend_motor.setPower(arm_extend_stick); 
            }
        }
        else
        {
            if (!ARM_IS_MOVING)
            {
                arm_extend_motor.setPower(0);
            }
        }
        
        if ((gamepad2.back && !saved_gamepad2.back) || arm_sensor.isPressed())
        {
            if (!ARM_IS_MOVING)
            {
                reset_arm_extend();
            }
        }
        
        //hang controls
        if (hang_left > 0)
        {
            if (arm_hang_motor.getCurrentPosition() < 12500)
            {
                arm_hang_motor.setPower(INTAKE_SPEED); 
            }
            else
            {
                arm_hang_motor.setPower(0); 
            }
        }
        else if (hang_right > 0)
        {
            arm_hang_motor.setPower(OUTTAKE_SPEED);
        }
        else 
        {
            arm_hang_motor.setPower(0.0);    
        }
        
        if (gamepad2.options && !saved_gamepad2.options)
        {
            override_checking = !override_checking;
        }
        
        //shoulder controls
        if (shoulder_stick != 0)
        {
            if (arm_extend_motor.getCurrentPosition() < 1000)
            {
                if (shoulder_stick < 0)
                {
                    shoulder_stick *= .7;
                }
                shoulder_motor.setPower(shoulder_stick); 
            }
        }
        else
        {
            shoulder_motor.setPower(0);
        }
        if ((gamepad2.start && !saved_gamepad2.start) || shoulder_sensor.isPressed())
        {
            reset_shoulder();
        }       
        if (gamepad1.dpad_left)
        {
            twist = .25;
        }
        else if (gamepad1.dpad_right)
        {
           twist = -.25;
        }
        else
        {
          twist  = -gamepad1.right_stick_x;  
        }
        
        if (strafe_left)
        {
            strafe = -.25;
        }
        else if (strafe_right)
        {
            strafe = .25;
        }
        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double new_strafe = strafe * Math.cos(heading) - drive * Math.sin(heading);
        double new_drive  = strafe * Math.sin(heading) + drive * Math.cos(heading); 
        if (DRIVER_MODE_FIELD == driver_mode)
        {
            drive = -new_drive;
            strafe = -new_strafe;
        }
        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++)
        {
            if ( max < Math.abs(speeds[i]) )
            {
                max = Math.abs(speeds[i]);
            }
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        /*
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++)
            {
                speeds[i] /= max;
            }
        }*/

        for(int i=0; i <speeds.length; i++)
        {
            if (max > 1)
            {
                speeds[i] /= max;
            }
            speeds[i] *= current_speed;
        }
        // apply the calculated values to the motors.
        move(speeds[0], speeds[1], speeds[2], speeds[3]);
        
        
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left front (%.2f), right front (%.2f)", speeds[0], speeds[1]);
        //telemetry.addData("Motors", "left rear  (%.2f), right rear  (%.2f)", speeds[2], speeds[3]);
        telemetry.addData("Current Arm Extend", current_arm_status_name);
        telemetry.addData("Current Arm Extend (pos)", arm_extend_motor.getCurrentPosition());
        telemetry.addData("Current Arm Extend (moving)", ARM_IS_MOVING);
        telemetry.addData("Current Arm Extend (pressed)", arm_sensor.isPressed());
        //telemetry.addData("Arm Extend stick", arm_extend_stick);
        //telemetry.addData("Arm Extend stick", -gamepad2.left_stick_y);
        telemetry.addData("Current Arm Hang", arm_hang_motor.getCurrentPosition());
        //telemetry.addData("Shoulder stick", shoulder_stick);
        telemetry.addData("Current Shoulder", shoulder_motor.getCurrentPosition());
        telemetry.addData("Current Speed", current_speed);
        telemetry.addData("Current Shoulder (pressed)", shoulder_sensor.isPressed());
        telemetry.addData("Override limit checking", override_checking);
        //telemetry.addData("Current LED", current_led_status);
        //telemetry.addData("heading",heading);
        telemetry.addData("heading (degrees)",heading_deg);
        telemetry.addData("Current Gripper", gripper_servo.getPosition());
        telemetry.addData("Current Gripper (status)", (current_gripper_status == GRIPPER_STATUS_OPEN) ? "OPEN" : "CLOSED");
        //telemetry.addData("Current Wrist", wrist_servo.getPosition());
        telemetry.update();
        
        //save all of the gamepad 
        saved_gamepad1.copy(gamepad1);
        saved_gamepad2.copy(gamepad2);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    public void stop_all_move()
    {
        move(0,0,0,0);
        arm_extend_motor.setPower(0);
        shoulder_motor.setPower(0);
    }
    public void move(double left_front, double right_front, double left_back, double right_back)
    {
        left_front_drive.setPower(left_front);
        left_rear_drive.setPower(left_back);
        
        right_front_drive.setPower(right_front);
        right_rear_drive.setPower(right_back);  
    }
    
    public void ExtendArm(double speed, double pos, double timeoutS)
    {
            ARM_IS_MOVING = true;
            current_target_position = (int)pos;

            int new_distance = arm_extend_motor.getCurrentPosition();
            speed = Math.abs(speed);
        
            arm_extend_motor.setTargetPosition(current_target_position);
            // Turn On RUN_TO_POSITION
            arm_extend_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            arm_extend_motor.setPower(speed);
    }

//this is the original, tight loop
    public void ExtendArmOrg(double speed, double pos, double timeoutS)
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

    public boolean IsShoulderLow()
    {
        if (override_checking)
        {
            return false;
        }
        else
        {
            return (shoulder_motor.getCurrentPosition() < SHOULDER_LOW_POSITION);
        }
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
            
        }
    }
    
    private void SetLED(int status)
    {
        switch(status)
        {
            case LED_STATUS_GREEN:
            {
                led_green.setState(true);
                led_red.setState(false);
                current_led_status = status;
                break;
            }
            case LED_STATUS_RED:
            {
                led_green.setState(false);
                led_red.setState(true);
                current_led_status = status;
                break;
            }
            case LED_STATUS_YELLOW:
            {
                led_green.setState(false);
                led_red.setState(false);
                current_led_status = status;
                break;
            }
            default:
            {
                current_led_status = LED_STATUS_OFF;
                led_green.setState(true);
                led_red.setState(true);
                break;
            }
        }
    }
}
