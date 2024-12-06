package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="EncodersAuto", group="BahTech")
public class EncodersAuto extends LinearOpMode {

    private final double kp = 0.001;
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    boolean objetivo = false;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            move (FL, FR, BL, BR, -10000);
            resetEncoder(FL, FR, BL, BR);
            move (FL, FR, BL, BR, 10000);
            resetEncoder(FL, FR, BL, BR);
            lateralMove(FL, FR, BL, BR, true, 300);
            resetEncoder(FL, FR, BL, BR);
            lateralMove(FL, FR, BL, BR, false, 300);
            resetEncoder(FL, FR, BL, BR);
            curve (FL, FR, BL, BR, true);
            resetEncoder(FL, FR, BL, BR);
            curve (FL, FR, BL, BR, false);
            resetEncoder(FL, FR, BL, BR);
            curveTotal(FL, FR, BL, BR, 500,true);
            resetEncoder(FL, FR, BL, BR);
            curveTotal (FL, FR, BL, BR, 500, false);
            resetEncoder(FL, FR, BL, BR);

        }

    }
    public void smoother(DcMotor motor, Double targetVelocity){
        int targetPos = motor.getTargetPosition();
        int currPos = motor.getCurrentPosition();

        double power = Math.abs(targetPos - currPos) * kp;
        if (power <= targetVelocity)
            motor.setPower(power);
        else
            motor.setPower(targetVelocity);

        if (power <= 0.003)
            motor.setPower(0);

    }
    public void move (DcMotor a, DcMotor b, DcMotor c, DcMotor d, int position) {
        a.setTargetPosition(position);
        b.setTargetPosition(position);
        c.setTargetPosition(position);
        d.setTargetPosition(position);
        a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        smoother(a,0.15);
        smoother(b,0.15);
        smoother(c,0.15);
        smoother(d,0.15);
        sleep(1200);
    }
    public void resetEncoder (DcMotor a, DcMotor b, DcMotor c, DcMotor d) {
        a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        c.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void curve (DcMotor a, DcMotor b, DcMotor c, DcMotor d, boolean direction) {
        if (direction) {
            a.setTargetPosition(-240);
            b.setTargetPosition(240);
            c.setTargetPosition(-240);
            d.setTargetPosition(240);
        } else {
            a.setTargetPosition(240);
            b.setTargetPosition(-240);
            c.setTargetPosition(240);
            d.setTargetPosition(-240);
        }
        a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        smoother(a,0.15);
        smoother(b,0.15);
        smoother(c,0.15);
        smoother(d,0.15);
        sleep(1200);
    }
    public void curveTotal (DcMotor a, DcMotor b, DcMotor c, DcMotor d, int position, boolean direction) {
        if (direction) {
            a.setTargetPosition(-position);
            b.setTargetPosition(position);
            c.setTargetPosition(-position);
            d.setTargetPosition(position);
        } else {
            a.setTargetPosition(position);
            b.setTargetPosition(-position);
            c.setTargetPosition(position);
            d.setTargetPosition(-position);
        }
        a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        smoother(a,0.15);
        smoother(b,0.15);
        smoother(c,0.15);
        smoother(d,0.15);
        sleep(2000);
    }
    public void lateralMove (DcMotor a, DcMotor b, DcMotor c, DcMotor d, boolean direction, int position) {
        if (direction) {
            a.setTargetPosition(position);
            b.setTargetPosition(-position);
            c.setTargetPosition(-position);
            d.setTargetPosition(position);
        } else {
            a.setTargetPosition(-position);
            b.setTargetPosition(position);
            c.setTargetPosition(position);
            d.setTargetPosition(-position);
        }
        a.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        c.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        smoother(a,0.15);
        smoother(b,0.15);
        smoother(c,0.15);
        smoother(d,0.15);
        sleep(1200);
    }


}