package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@TeleOp(name = "Encoder Driver")
public class encoderdrive extends LinearOpMode {

    final private double R2D3 = Math.sqrt(2);
    public DcMotorEx backleft;
    public DcMotorEx backright;
    public DcMotorEx frontleft;
    public DcMotorEx frontright;

    private BNO055IMU imu;

    public boolean driveinit() {
        backleft = (DcMotorEx)(hardwareMap.get(DcMotor.class, "backleft"));
        backright = (DcMotorEx)(hardwareMap.get(DcMotor.class, "backright"));
        frontleft = (DcMotorEx)(hardwareMap.get(DcMotor.class, "frontleft"));
        frontright = (DcMotorEx)(hardwareMap.get(DcMotor.class, "frontright"));
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setPositionPIDFCoefficients(2.5);
        backright.setPositionPIDFCoefficients(2.5);
        frontleft.setPositionPIDFCoefficients(2.5);
        frontright.setPositionPIDFCoefficients(2.5);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.mode = BNO055IMU.SensorMode.IMU;
        params.loggingEnabled = false;

        imu.initialize(params);

        telemetry.addData("Calibrating", "");
        telemetry.update();

        while (!imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Ready", "");
        telemetry.update();

        return backleft != null && backright != null && frontleft != null && frontright != null;

    }

    public void debug() {
        telemetry.addData("", "BL: %s <- %s || %s", backleft.getTargetPosition(), backleft.getCurrentPosition(), backleft.getMode());
        telemetry.addData("", "BR: %s <- %s || %s", backright.getTargetPosition(), backright.getCurrentPosition(), backright.getMode());
        telemetry.addData("", "FL: %s <- %s || %s", frontleft.getTargetPosition(), frontleft.getCurrentPosition(), frontleft.getMode());
        telemetry.addData("", "FR: %s <- %s || %s", frontright.getTargetPosition(), frontright.getCurrentPosition(), frontright.getMode());
    }

    public void runOffset(double targetx, double targety) {

        int tx = (int) (targetx / (75.0 * Math.PI) * 560.0);
        int ty = (int) (targety / (75.0 * Math.PI) * 560.0);

        backleft.setTargetPosition(-tx + ty);
        ((DcMotorEx) backleft).setTargetPositionTolerance(10);
        backright.setTargetPosition(tx + ty);
        ((DcMotorEx) backright).setTargetPositionTolerance(10);
        frontleft.setTargetPosition(tx + ty);
        ((DcMotorEx) frontleft).setTargetPositionTolerance(10);
        frontright.setTargetPosition(-tx + ty);
        ((DcMotorEx) frontright).setTargetPositionTolerance(10);

        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (busy()) {
            telemetry.addData("", "busy: %s", busy());
            telemetry.update();
        }
    }

    public void runOffset(double targetx, double targety, double r) {


        int x = (int) (targetx / (75.0 * Math.PI) * 560.0);
        int y = (int) (targety / (75.0 * Math.PI) * 560.0);

        int tx = (int) (((x*Math.cos(Math.toRadians(r))) - (y*Math.sin(Math.toRadians(r)))));
        int ty = (int) (((x*Math.sin(Math.toRadians(r))) + (y*Math.cos(Math.toRadians(r)))));

        setTurn(r);

        reset();

        backleft.setTargetPosition(-tx + ty);
        ((DcMotorEx) backleft).setTargetPositionTolerance(40);
        backright.setTargetPosition(tx + ty);
        ((DcMotorEx) backright).setTargetPositionTolerance(40);
        frontleft.setTargetPosition(tx + ty);
        ((DcMotorEx) frontleft).setTargetPositionTolerance(40);
        frontright.setTargetPosition(-tx + ty);
        ((DcMotorEx) frontright).setTargetPositionTolerance(40);

        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backleft.setPower(0.4);
        backright.setPower(0.4);
        frontleft.setPower(0.4);
        frontright.setPower(0.4);

        while (busy()) {
            telemetry.addData("", "busy: %s targetx %s targety %s tx %s ty %s", busy(), targetx, targety, tx, ty);
            telemetry.addData("", "%s %s", frontleft.getTargetPosition(), frontright.getTargetPosition());
            telemetry.addData("", "%s %s", backleft.getTargetPosition(), backright.getTargetPosition());
            telemetry.update();
            sleep(20);
            idle();
        }

    }

    public void reset() {
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setPositionPIDFCoefficients(2.5);
        backright.setPositionPIDFCoefficients(2.5);
        frontleft.setPositionPIDFCoefficients(2.5);
        frontright.setPositionPIDFCoefficients(2.5);
    }

    public float heading(int x) {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle * -1 + 180 + x;
    }

    public void setTurn(double theta) {

        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double target = (heading(0) + theta) % 360;
        int a = (int) (Math.abs(theta)/theta);

        backleft.setVelocity(490*a);
        backright.setVelocity(-490*a);
        frontleft.setVelocity(490*a);
        frontright.setVelocity(-490*a);

        while (heading(-1) > target || heading(1) < target) {
            telemetry.addData("", "%s %s", heading(0), a);
            telemetry.update();
            sleep(10);
        }

        backleft.setVelocity(0);
        backright.setVelocity(0);
        frontleft.setVelocity(0);
        frontright.setVelocity(0);

    }

//    public void drive() {
//        backleft.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 3.0 * 2.0);
//        backright.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)  / 3.0 * 2.0);
//        frontleft.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / 3.0 * 2.0);
//        frontright.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / 3.0 * 2.0);
//    }

    public void runCourse() {
        runOffset(0.0, 0.0);
    }

    public boolean busy() {
        return (backleft.isBusy() || backright.isBusy() || frontleft.isBusy() || frontright.isBusy());
    }

    @Override
    public void runOpMode() {

        boolean isinput = false;
        boolean wasinput = false;
        driveinit();
        waitForStart();
        reset();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("", "busy: %s || isinput: %s || wasinput: %s", busy(), isinput, wasinput);
                debug();
                telemetry.update();
                if ((isinput = gamepad1.a) && !wasinput) {
                    runOffset(600, 0.0, 0.0);
                } else if ((isinput = gamepad1.b) && !wasinput) {
                    setTurn(45.0);
                    reset();
                } else if ((isinput = gamepad1.x) && !wasinput) {
                    setTurn(-45.0);
                    reset();
                } else if ((isinput = gamepad1.y) && !wasinput) {
                    runOffset(100.0, 0.0);
                    reset();
                } else if ((isinput = gamepad1.start) && !wasinput) {
                    runOffset(-100.0, 0.0);
                    reset();
                } else if ((isinput = gamepad1.back) && !wasinput) {
                    runOffset(0.0, -27.0*25.4/2.0);
                    reset();
                }
                wasinput = isinput;
            }
        }
    }

}

