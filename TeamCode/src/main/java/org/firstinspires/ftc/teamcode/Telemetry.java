package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class Telemetry extends OpMode {
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx arm;
    private DcMotorEx intakel;
    private DcMotorEx intaker;
    private ServoImplEx clawl;
    private ServoImplEx clawr;
    private Servo drone;
    private ServoImplEx rotate;
    private BNO055IMU imu;
    @Override
    public void init() {
        intakel = hardwareMap.get(DcMotorEx.class,"intakel");
        intaker = hardwareMap.get(DcMotorEx.class,"intaker");
        arm = hardwareMap.get(DcMotorEx.class,"arm");
        clawl = hardwareMap.get(ServoImplEx.class,"clawl");
        clawr = hardwareMap.get(ServoImplEx.class,"clawr");
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        drone = hardwareMap.get(Servo.class,"drone");
        rotate = hardwareMap.get(ServoImplEx.class, "rotate");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawl.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawr.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rotate.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
        telemetry.addData("front left power", leftFront.getPower());
        telemetry.addData("front right power", rightFront.getPower());
        telemetry.addData("back left power", leftBack.getPower());
        telemetry.addData("back right power", rightBack.getPower());

        telemetry.addData("left claw servo position", clawl.getPosition());
        telemetry.addData("right claw servo position", clawr.getPosition());
        telemetry.addData("drone servo position", drone.getPosition());
        telemetry.addData("Front right Encoder ticks", rightFront.getCurrentPosition());
        telemetry.addData("back left encoder", leftBack.getCurrentPosition());
        telemetry.addData("Back Right Encoder (para) ticks", rightBack.getCurrentPosition());
        telemetry.addData("imu yaw",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        telemetry.addData("arm encoder", arm.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void stop() {

    }
}
