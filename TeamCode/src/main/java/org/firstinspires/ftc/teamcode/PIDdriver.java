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

@TeleOp(name = "PID TESTS")
public class PIDdriver extends LinearOpMode {

    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;

    private BNO055IMU imu;

    private double firstangle;

    final private double MAX_RPM = 6000.0;

    final private double TICKS_PER_REV = 28.0;

    final private double GEAR = 20.0;

    final private double MAX_TICKS_PER_SEC = MAX_RPM / 60 * TICKS_PER_REV * GEAR;

    private double P = MAX_TICKS_PER_SEC / 80000;
    private double D = 4.0;
    
    private double targetx = 0;
    
    private double targety = 0;

    private double ebl = 0.0;
    private double ebr = 0.0;
    private double efl = 0.0;
    private double efr = 0.0;

    public boolean driveinit() {

        backleft = (DcMotorEx) hardwareMap.get(DcMotor.class, "backleft");
        backright = (DcMotorEx) hardwareMap.get(DcMotor.class, "backright");
        frontleft = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontleft");
        frontright = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontright");

        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);

        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return backleft != null && frontleft != null && backright != null && frontright != null;

    }

    public boolean imuinit() {

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.loggingEnabled = true;
        params.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(params);

        while (!imu.isGyroCalibrated() && opModeInInit()) {
            telemetry.addData("IMU:", "INITIALIZING");
            telemetry.update();
            sleep(20);
            idle();
        }

        firstangle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

        return true;

    }
    
    public void pid() {
        double rx = targetx / (75.0 * Math.PI) * GEAR * TICKS_PER_REV;
        double ry = targety / (75.0 * Math.PI) * GEAR * TICKS_PER_REV;

        double cebl = (ry - backleft.getCurrentPosition()) + (-rx - backleft.getCurrentPosition());
        double cebr = (ry - backright.getCurrentPosition()) + (rx - backright.getCurrentPosition());
        double cefl = (ry - frontleft.getCurrentPosition()) + (rx - frontleft.getCurrentPosition());
        double cefr = (ry - frontright.getCurrentPosition()) + (-rx - frontright.getCurrentPosition());

        double pbl = cebl * P;
        double pbr = cebr * P;
        double pfl = cefl * P;
        double pfr = cefr * P;

        double dbl = (cebl - ebl) * D;
        double dbr = (cebr - ebl) * D;
        double dfl = (cefl - ebl) * D;
        double dfr = (cefr - ebl) * D;

        ebl = cebl;
        ebr = cebr;
        efl = cefl;
        efr = cefr;

        backleft.setVelocity(pbl + dbl);
        backright.setVelocity(pbr + dbr);
        frontleft.setVelocity(pfl + dfl);
        frontright.setVelocity(pfr + dfr);

        telemetry.addData("", "rx: %s || ry: %s", rx, ry);
        telemetry.update();

    }

    public void debug() {
        telemetry.addData("", "BL: %s <- %s || %s", backleft.getTargetPosition(), backleft.getCurrentPosition(), backleft.getMode());
        telemetry.addData("", "BR: %s <- %s || %s", backright.getTargetPosition(), backright.getCurrentPosition(), backright.getMode());
        telemetry.addData("", "FL: %s <- %s || %s", frontleft.getTargetPosition(), frontleft.getCurrentPosition(), frontleft.getMode());
        telemetry.addData("", "FR: %s <- %s || %s", frontright.getTargetPosition(), frontright.getCurrentPosition(), frontright.getMode());
    }

    @Override
    public void runOpMode() {
        
        if (driveinit()) {
            telemetry.addData("DRIVE:", "READY");
            telemetry.update();
        }
        if (imuinit()) {
            telemetry.addData("DRIVE:", "READY");
            telemetry.addData("IMU:", "READY");
            telemetry.update();
        }
        
        waitForStart();
        if (opModeIsActive()) {
            
            int input = 0;

            boolean go = true;
            
            while (opModeIsActive()) {

                input = Math.max(input - 1, 0);
                
                if (gamepad1.a && input == 0) {
                    targety += -1000;
                    input += 20;
                } else if (gamepad1.b && input == 0) {
                    targety += 1000;
                    input += 20;
                } else if (gamepad1.x && input == 0) {
                    go = !go;
                    input += 20;
                } else if (gamepad1.y && input == 0) {
                    targetx += 200;
                    targety += 200;
                    input += 20;
                }

                if (go) pid();
            }
        }

    }

}
