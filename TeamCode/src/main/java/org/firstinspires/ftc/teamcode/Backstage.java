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

import java.util.ArrayList;

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
    boolean backstage = true;
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

        /*dcArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
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
            telemetry.addData("square: ", "blue");
            telemetry.addData("circle: ", "red");
            telemetry.addData("triangle: ", "backstage");
            telemetry.addData("cross: ", "audience");
            telemetry.update();
            if (gamepad1.square) {
                red = false;
            }
            if (gamepad1.circle) {
                red = true;
            }
            if (gamepad1.triangle){
                backstage = true;
            }
            if(gamepad1.cross){
                backstage = false;
            }

            arm.loop();
        }
        short pixyBytes1; // need this
        short pixyBytes2; // need this
        short pixyBytes3; // need this
        short pixyBytes4; // need this
        short pixyBytes5;
        while(!isStopRequested() && !isStarted()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("red side? ", red);
            telemetry.addData("backstage? ", backstage);

            pixyBytes1 = pixy.readShort(0x51, 8); // need this
            telemetry.addData("number of Signature 1", pixyBytes1); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1); // need this
            pixyBytes2 = pixy.readShort(0x52, 8);
            telemetry.addData("number of Signature 2", pixyBytes2); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2); // need this
            pixyBytes3 = pixy.readShort(0x53, 8);
            telemetry.addData("number of Signature 3", pixyBytes3); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3); // need this
            pixyBytes4 = pixy.readShort(0x54, 8);
            telemetry.addData("number of Signature 4", pixyBytes4); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4); // need this
            pixyBytes5 = pixy.readShort(0x55, 8);
            telemetry.addData("number of Signature 5", pixyBytes5); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5); // need this
            telemetry.update();
            arm.loop();
        }
        // Wait for driver to press start
        waitForStart();

        rotate.setPosition(0.8);
        //close claw

        //Pixy look for team prop
        int redAvg = 0;
        int blueAvg = 0;
        int numRedSigs = 2; //IF YOU COMMENT STUFF OUT, CHANGE THIS
        int numBlueSigs = 2; //IF YOU COMMENT STUFF OUT, CHANGE THIS
        int i = 1;
        int loops = 0;
        ArrayList<Integer> posR = new ArrayList<Integer>();
        ArrayList<Integer> posB = new ArrayList<Integer>();
        while (i < 201 && opModeIsActive()) {
            pixyBytes1 = pixy.readShort(0x51, 2); // need this
            if (pixyBytes1!=0){
                redAvg = redAvg + pixyBytes1;
                loops++;
                //posR.add((int)pixyBytes1[1]);
            }
            telemetry.addData("number of Signature 1", pixyBytes1); // need this
            telemetry.addData("x position of largest block of sig 1", pixyBytes1); // need this
            pixyBytes2 = pixy.readShort(0x52, 2);
            if (pixyBytes2!=0) {
                blueAvg = blueAvg + pixyBytes2;
                loops++;
                //posB.add((int)pixyBytes2[1]);
            }
            telemetry.addData("number of Signature 2", pixyBytes2); // need this
            telemetry.addData("x position of largest block of sig 2", pixyBytes2); // need this

            pixyBytes3 = pixy.readShort(0x53, 2);
            if (pixyBytes3!=0) {
                redAvg = redAvg + pixyBytes3;
                loops++;
                //posR.add((int)pixyBytes3[1]);
            }
            telemetry.addData("number of Signature 3", pixyBytes3); // need this
            telemetry.addData("x position of largest block of sig 3", pixyBytes3); // need this

            pixyBytes4 = pixy.readShort(0x54, 2);
            if (pixyBytes4!=0) {
                blueAvg = blueAvg + pixyBytes4;
                loops++;
                //posB.add((int)pixyBytes4[1]);
            }
            telemetry.addData("number of Signature 4", pixyBytes4); // need this
            telemetry.addData("x position of largest block of sig 4", pixyBytes4); // need this

            pixyBytes5 = pixy.readShort(0x55, 2);
            if (pixyBytes5!=0) {
               redAvg = redAvg + pixyBytes5;
                  loops++;
            }
            telemetry.addData("number of Signature 5", pixyBytes5); // need this
            telemetry.addData("x position of largest block of sig 5", pixyBytes5); // need this
            i++;
            telemetry.addData("num times looped", loops);
            telemetry.update();

        }//close pixy for loop
        if(red == true){
            int redPos = redAvg / (loops + 1);
            telemetry.addData("redpos", redPos);
            telemetry.update();
            if (redPos < 350) {
                position = 'L';
            } else if (redPos > 400) {
                position = 'R';
            } else {
                position = 'C';
            }
           /* int redSum = 0;
            for(int r = 0; r < posR.size(); r++){
                redSum += posR.get(r);
                telemetry.addData("sum", redSum);
            }
            int avgr = redSum / (posR.size() + 1);
            telemetry.addData("division", avgr);
            telemetry.update();
            if (20<avgr && avgr<90) {
                position = 'L';
            } else if (-20 > avgr && -90 < avgr) {
                position = 'R';
            } else {
                position = 'C';
            }
            */
        }else if(red == false){
            /*int bluePos = 0;
            telemetry.addData("b size", posB.size());
            telemetry.update();
            for(int b = 0; b < posB.size(); b++){
                bluePos += posB.get(b);
                telemetry.addData("for loop", bluePos);
                telemetry.update();
            }
            int avgb = bluePos / (posB.size()+1);
            if (20<avgb && avgb<90) {
                position = 'L';
            } else if (-20 > avgb && -90 < avgb) {
                position = 'R';
            } else {
                position = 'C';
            }*/
            int bluePos = blueAvg / (loops + 1);
            telemetry.addData("bluepos", bluePos);
            telemetry.update();
            if (bluePos < 350) {
                position = 'L';
            } else if (bluePos > 400) {
                position = 'R';
            } else {
                position = 'C';
            }
        }
        //runtime.reset();
        //while (runtime.seconds()<5 && opModeIsActive()){}
        /*while (dcArm.getCurrentPosition()>-700&& opModeIsActive()){
            dcArm.setPower(-.5);
        }
        dcArm.setPower(0);*/
        //Robot needs to drive and move forward like 24in ish
        drive(32, directions.FORWARD, 0.25);

        //If Drop pixel at left: turn left 90 degrees then open claw then turn right to get back on track.
        if (position == 'L'){
            turn(90, directions.LEFT, 0.25);
            drive(1.5, directions.FORWARD, 0.25);
            arm.setTargetPos(-100);
            clawr.setPosition(0.7);
            drive(-3, directions.FORWARD, 0.25);
            if(backstage == true && red == true){
                turn(-90, directions.RIGHT, .25);
                drive(30, directions.FORWARD, .25);
                clawl.setPosition(0.6);
                drive(-3, directions.FORWARD, .25);
                if (red == true) {
                    drive(-100, directions.SIDE, .25);
                } else if (red == false) {
                    drive(100, directions.SIDE, .25);
                }
            }
        }
        else if (position == 'C'){
            // Drop pixel at center: drive past then turn around 180 degrees and then drop pixel and then turn another 180 degrees.
            drive(-1, directions.FORWARD, 0.25);
            arm.setTargetPos(-100);
            //open
            clawr.setPosition(0.7);
            drive(-4, directions.FORWARD, 0.25);
            if(backstage == true){
                if(red == true){
                    turn(-90, directions.RIGHT, .25);
                }
                else {
                    turn(90, directions.RIGHT, .25);
                }
                drive(30, directions.FORWARD, .25);
                clawl.setPosition(0.6);
                drive(-3, directions.FORWARD, .25);
                if (red == true) {
                    drive(-100, directions.SIDE, .25);
                } else if (red == false) {
                    drive(100, directions.SIDE, .25);
                }
            }
        }
        else if(position== 'R'){
            //Then turn right 90 degrees drop pixel at right
            turn(-90, directions.RIGHT, .25);
            drive(1.5, directions.FORWARD, .25);
            arm.setTargetPos(-100);
            clawr.setPosition(0.7);
            drive(-3, directions.FORWARD, .25);
            if(backstage == true && red == false){
                turn(90, directions.RIGHT, .25);
                drive(30, directions.FORWARD, .25);
                clawl.setPosition(0.6);
                drive(-3, directions.FORWARD, .25);
                if (red == true) {
                    drive(-100, directions.SIDE, .25);
                } else if (red == false) {
                    drive(100, directions.SIDE, .25);
                }
            }

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        //PoseStorage.currentPose = drive.getPoseEstimate();

    } //close runOpMode()
} // close entire class
