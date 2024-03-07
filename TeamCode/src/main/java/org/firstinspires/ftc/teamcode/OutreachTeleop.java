package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//list of controller function here: https://docs.google.com/document/d/1mTPaoFG1fvQqmZDU4-IlvfgqJ6lwRfTvbBcxseAkrCM/edit?usp=sharing
@TeleOp
public class OutreachTeleop extends OpMode {
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx dcArm;
    private DcMotorEx intakel;
    private DcMotorEx intaker;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private Servo drone;
    private ServoImplEx rotate;
    private BNO055IMU imu;
    private Pixy pixy;
    private PIDF_Arm arm;
    @Override
    public void init() {
        intakel = hardwareMap.get(DcMotorEx.class,"intakel");
        intaker = hardwareMap.get(DcMotorEx.class,"intaker");
        dcArm = hardwareMap.get(DcMotorEx.class,"arm");
        clawl = hardwareMap.get(ServoImplEx.class,"clawl");
        clawr = hardwareMap.get(ServoImplEx.class,"clawr");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        drone = hardwareMap.get(Servo.class,"drone");
        rotate = hardwareMap.get(ServoImplEx.class, "rotate");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        pixy = hardwareMap.get(Pixy.class, "pixy");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rotate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawl.setDirection(Servo.Direction.REVERSE);
        clawr.setDirection(Servo.Direction.REVERSE);
        rotate.setDirection(Servo.Direction.REVERSE);

        dcArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm = new PIDF_Arm(dcArm, telemetry);
        arm.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
    }
    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            //outtake
            //arm.setTargetPos(-4000);

            /*arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (1500>arm.getCurrentPosition()&& arm.getCurrentPosition()>0) {
                arm.setPower(-1);
            } else if(1500<arm.getCurrentPosition() && arm.getCurrentPosition()<2000) {
                arm.setPower(-.5);
            }else if (arm.getCurrentPosition()> 2000){
                arm.setPower(0);
            }*/

        } else if (gamepad2.dpad_down) {
            //intake
            //arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //arm.setTargetPos(-20);
             /*
            if(arm.getCurrentPosition() > 100) {
                arm.setPower(0.75);
            } else{
                arm.setPower(0);
            }
            */
        } else if (gamepad2.dpad_right) {
            //transport
            //arm.setTargetPos(-3000);
            /*if(arm.getCurrentPosition() < 1200) {
                arm.setPower(-1);
            } else if(arm.getCurrentPosition() > 1500){
                arm.setPower(0.75);
            } else{
                arm.setPower(0);
            }*/
        } else if (gamepad2.right_bumper) {
            //manual down
            dcArm.setPower(1);
        } else if (gamepad2.right_trigger > 0.1) {
            //manual up
            dcArm.setPower(-gamepad2.right_trigger);
        } else /*if(arm.isBusy() == false)*/ {
            dcArm.setPower(0);
        }
        if (gamepad2.square) {
            //open
            clawl.setPosition(0.6);
        } else if (gamepad2.circle) {
            //open
            clawr.setPosition(.7);
        } else if (gamepad2.cross) {
            //open
            clawl.setPosition(0.6);
            clawr.setPosition(0.7);
        } else if (gamepad1.square) {
            //open
            clawl.setPosition(0.6);
        } else if (gamepad1.circle) {
            //open
            clawr.setPosition(.7);
        } else if (gamepad1.cross) {
            //open
            clawl.setPosition(0.6);
            clawr.setPosition(0.7);
        } else {
            //close
            clawl.setPosition(0.7);
            clawr.setPosition(0.6);
        }

        if (gamepad2.left_trigger > 0.05) {
            //up above field (actually closer to flat)
            rotate.setPosition(0.4);
        } else if (gamepad2.left_bumper) {

            //down below field
            rotate.setPosition(.9);
        } else if (arm.getCurrentPos() < 3000) {

            rotate.setPosition(0.8);
        } else if (arm.getCurrentPos() >3000){
            rotate.setPosition(0.4);
        } else{
            //flat on field (actually closer to above)
            rotate.setPosition(0.8);
        }
        if (gamepad2.triangle) {
            drone.setPosition(0.5);
        } else {
            drone.setPosition(1);
            //1 is when not pushed and .5 is when pushed
            // 1 is holding rubberband .5 is out
        }

        //driving code from gm0.org
        double ly;
        double lx;
        double rx;


            ly = -gamepad1.left_stick_y * 0.5;
            lx = gamepad1.left_stick_x * 0.5;
            rx = gamepad1.right_stick_x * 0.5;
        
        leftFront.setPower(ly + lx + rx);
        leftBack.setPower(ly - lx + rx);
        rightFront.setPower(-ly + lx + rx);
        rightBack.setPower(ly + lx - rx);

        /*
        //field centric code from gm0.org
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        double botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
         */

        //arm.loop(); //this always needs to be here for arm to work

        telemetry.addData("front left power", leftFront.getPower());
        telemetry.addData("front right power", rightFront.getPower());
        telemetry.addData("back left power", leftBack.getPower());
        telemetry.addData("back right power", rightBack.getPower());

        telemetry.addData("left claw servo position", clawl.getPosition());
        telemetry.addData("right claw servo position", clawr.getPosition());
        telemetry.addData("drone servo position", drone.getPosition());
        telemetry.addData("Front Left Encoder (perp) ticks", leftFront.getCurrentPosition());
        telemetry.addData("Negative Back Right Encoder (para) ticks", -rightBack.getCurrentPosition());
        telemetry.addData("Front Left Encoder (perp) inches", encoderTicksToInches(leftFront.getCurrentPosition()));
        telemetry.addData("Negative Back Right Encoder (para) inches", encoderTicksToInches(-rightBack.getCurrentPosition()));
        telemetry.addData("imu yaw",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        telemetry.addData("arm encoder", arm.getCurrentPos());
        short pixyBytes1 = pixy.readShort(0x51, 5); // need this
        telemetry.addData("number of Signature 1", pixyBytes1); // need this
        telemetry.addData("x position of largest block of sig 1", pixyBytes1); // need this
        short pixyBytes2 = pixy.readShort(0x52, 2); // need this
        telemetry.addData("number of Signature 2", pixyBytes2); // need this
        telemetry.addData("x position of largest block of sig 2", pixyBytes2); // need this
        telemetry.update();
    }
    @Override
    public void stop() {

    }
    public static double X_MULTIPLIER = 0.9787360469; // Multiplier in the X direction: 1.005395271
    public static double Y_MULTIPLIER = 0.982667947; // Multiplier in/ the Y direction
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
