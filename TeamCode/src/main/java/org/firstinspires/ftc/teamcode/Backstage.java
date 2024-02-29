package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class Backstage extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private DcMotorEx dcArm;
    private PIDF_Arm arm;
    private BNO055IMU imu;
    private Servo rotate;
    private enum directions{
        FORWARD,
        SIDE,
        LEFT,
        RIGHT
    }
    public static double X_MULTIPLIER = 0.9787360469; // Multiplier in the X direction: 1.005395271
    public static double Y_MULTIPLIER = 0.982667947; // Multiplier in/ the Y direction
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    char position;
    ElapsedTime runtime = new ElapsedTime();
    private Pixy pixy; // need this
    double lastParEncoder_in = 0;
    double lastPerpEncoder_in = 0;
    double lastHeading = 0;
    double xPos_in;
    double yPos_in;
    double heading;
    boolean red = false;
    double desiredDirection;
    //for forward: left side need to be negative
    public void drive(double inches, directions dir, double power) {
        lastParEncoder_in = encoderTicksToInches(-rightBack.getCurrentPosition());
        lastPerpEncoder_in = encoderTicksToInches(leftFront.getCurrentPosition());
        xPos_in = encoderTicksToInches(-rightBack.getCurrentPosition());
        yPos_in = encoderTicksToInches(leftFront.getCurrentPosition());

        if(dir == directions.FORWARD) {
            while (((Math.abs(xPos_in - (inches + lastParEncoder_in))) > 0.5) && opModeIsActive()) {
                xPos_in = encoderTicksToInches(-rightBack.getCurrentPosition());
                desiredDirection = (xPos_in - (inches + lastParEncoder_in)) / (Math.abs(xPos_in - (inches + lastParEncoder_in)));

                leftFront.setPower(-desiredDirection * power);
                leftBack.setPower(-desiredDirection * power);
                rightFront.setPower(desiredDirection * power);
                rightBack.setPower(desiredDirection * power);

                telemetry.addData("negative Right Back Encoder (para) ticks", -rightBack.getCurrentPosition());
                telemetry.addData("left front Encoder (perp) ticks", leftFront.getCurrentPosition());
                telemetry.addData("negative right back Encoder (para) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));
                telemetry.addData("Negative left front Encoder (perp) inches", encoderTicksToInches(leftFront.getCurrentPosition()));

                telemetry.addData("offset from position:", xPos_in - (inches + lastParEncoder_in));
                telemetry.update();
            }
        }
        //positive inches is left
        if(dir == directions.SIDE) {
            while (((Math.abs(yPos_in - (inches + lastPerpEncoder_in))) > 0.5) && opModeIsActive()) {
                yPos_in = encoderTicksToInches(leftFront.getCurrentPosition());
                desiredDirection = (yPos_in - (inches + lastPerpEncoder_in)) / (Math.abs(yPos_in - (inches + lastPerpEncoder_in)));

                leftFront.setPower(desiredDirection * power);
                leftBack.setPower(-desiredDirection * power);
                rightFront.setPower(desiredDirection * power);
                rightBack.setPower(-desiredDirection * power);

                telemetry.addData("right back Encoder (perp) ticks", -rightBack.getCurrentPosition());
                telemetry.addData("right back Encoder (perp) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));

                telemetry.addData("offset from position:", yPos_in - (inches + lastPerpEncoder_in));
                telemetry.update();
            }
        }
    }
    //to rotate counterclockwise (increasing imu) RB should be the only negative power
    //positive degrees is clockwise
    public void turn(double degrees, directions dir, double power) {
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        runtime.reset();

        while (((Math.abs(degrees - heading)) > 3) && opModeIsActive()) {
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            desiredDirection = (degrees - heading) / (Math.abs(degrees - heading));

            leftFront.setPower(-desiredDirection * power);
            leftBack.setPower(-desiredDirection * power);
            rightFront.setPower(-desiredDirection * power);
            rightBack.setPower(-desiredDirection * power);

            telemetry.addData("degrees:", degrees);
            telemetry.addData("heading", heading);
            telemetry.addData("desired direction", desiredDirection);
            telemetry.addData("offset from position:", degrees - heading);
            telemetry.update();
        }
        /*if (dir == directions.LEFT) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle< degrees && opModeIsActive()) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(power);

                telemetry.addData("Yaw degrees: ",  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.update();
            }
        }
        if (dir == directions.RIGHT) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle > -degrees && opModeIsActive()) {
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(-power);

                telemetry.addData("Yaw degrees: ",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
                telemetry.update();
            }
        }*/
    }
    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        dcArm = hardwareMap.get(DcMotorEx.class,"arm");
        clawl = hardwareMap.get(ServoImplEx.class,"clawl");
        clawr = hardwareMap.get(ServoImplEx.class,"clawr");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rotate = hardwareMap.get(Servo.class, "rotate");
        pixy = hardwareMap.get(Pixy.class, "pixy"); // need this

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        PIDF_Arm arm = new PIDF_Arm(dcArm, telemetry);
        arm.init();

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawl.setDirection(Servo.Direction.REVERSE);
        clawr.setDirection(Servo.Direction.REVERSE);
        rotate.setDirection(Servo.Direction.REVERSE);

        clawl.setPosition(0.7);
        clawr.setPosition(0.6);

        arm.setTargetPos(-400);
        rotate.setPosition(0.4);
        //arm.setPower(1);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //voodoo
        while(gamepad1.left_bumper && !isStopRequested()) {
            telemetry.addData("x or square: ", "blue");
            telemetry.addData("b or circle: ", "red");
            telemetry.update();
            if (gamepad1.x) {
                red = false;
            }
            if (gamepad1.b) {
                red = true;
            }
            arm.loop();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.addData("red side? ", red);
        byte[] pixyBytes1; // need this
        byte[] pixyBytes2; // need this
        byte[] pixyBytes3; // need this
        byte[] pixyBytes4; // need this
        byte[] pixyBytes5;
        while(!isStopRequested() && !isStarted()) {
            pixyBytes1 = pixy.readShort(0x51, 5); // need this
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this
            pixyBytes2 = pixy.readShort(0x52, 2);
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this
            pixyBytes3 = pixy.readShort(0x53, 2);
            telemetry.addData("number of Signature 3", pixyBytes3[0]); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3[1]); // need this
            pixyBytes4 = pixy.readShort(0x54, 2);
            telemetry.addData("number of Signature 4", pixyBytes4[0]); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4[1]); // need this
            pixyBytes5 = pixy.readShort(0x55, 2);
            telemetry.addData("number of Signature 5", pixyBytes5[0]); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5[1]); // need this
            telemetry.update();
            arm.loop();
        }
        // Wait for driver to press start
        waitForStart();
        arm.setTargetPos(-20);
        rotate.setPosition(0.8);
        //close claw

        //Pixy look for team prop
        int redAvg = 0;
        int blueAvg = 0;
        int numRedSigs = 3; //IF YOU COMMENT STUFF OUT, CHANGE THIS
        int numBlueSigs = 2; //IF YOU COMMENT STUFF OUT, CHANGE THIS
        for (int i = 1; i < 101; i++) {
            pixyBytes1 = pixy.readShort(0x51, 5); // need this
            redAvg = redAvg + pixyBytes1[1];
            telemetry.addData("number of Signature 1", pixyBytes1[0]); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1[1]); // need this
            pixyBytes2 = pixy.readShort(0x52, 2);
            blueAvg = blueAvg + pixyBytes2[1];
            telemetry.addData("number of Signature 2", pixyBytes2[0]); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2[1]); // need this
            pixyBytes3 = pixy.readShort(0x53, 2);
            redAvg = redAvg + pixyBytes3[1];
            telemetry.addData("number of Signature 3", pixyBytes3[0]); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3[1]); // need this
            pixyBytes4 = pixy.readShort(0x54, 2);
            blueAvg = blueAvg + pixyBytes4[1];
            telemetry.addData("number of Signature 4", pixyBytes4[0]); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4[1]); // need this
            pixyBytes5 = pixy.readShort(0x55, 2);
            redAvg = redAvg + pixyBytes5[1];
            telemetry.addData("number of Signature 5", pixyBytes5[0]); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5[1]); // need this
            telemetry.update();
            if(red == true){
                int redPos = redAvg / (i*numRedSigs);
                if (redPos <= 100) {
                    position = 'L';
                } else if (redPos > 100 && redPos < 200) {
                    position = 'C';
                } else {
                    position = 'R';
                }
            }else if(red == false){
                int bluePos = blueAvg / (i*numBlueSigs);
                if(bluePos <= 100){
                    position = 'L';
                } else if (bluePos > 100 && bluePos < 200){
                    position = 'C';
                } else {
                    position = 'R';
                }
            }
        }//close pixy for loop

        //Robot needs to drive and move forward like 24in ish
        drive(32, directions.FORWARD, 0.25);

        //If Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
            if (position == 'L'){
                turn(90, directions.LEFT, 0.25);
                drive(1.5, directions.FORWARD, 0.25);
                clawr.setPosition(0.7);
                drive(-3, directions.FORWARD, 0.25);
            }
            else if (position == 'C'){
                // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
                drive(-1, directions.FORWARD, 0.25);
                //open
                clawr.setPosition(0.7);
                drive(-4, directions.FORWARD, 0.25);
            }
            else if(position== 'R'){
                //Then turn right 90 degrees drop pixel at right
                turn(-90, directions.RIGHT, .25);
                drive(1.5, directions.FORWARD, .25);
                clawr.setPosition(0.7);
                drive(-3, directions.FORWARD, .25);
            }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        //PoseStorage.currentPose = drive.getPoseEstimate();

    } //close runOpMode()
} // close entire class
