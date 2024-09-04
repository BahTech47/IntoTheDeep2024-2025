package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="ControleVelocidade", group="BahTech")
public class ControleVelocidade extends LinearOpMode {

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    @Override
    public void runOpMode() throws InterruptedException {

        int aPosition = 300;
        int bPosition = 0;

        double VFL = 0.1;
        double VFR = 0.1;
        double VBL = 0.1;
        double VBR = 0.1;
        double temporaria = 0;

        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            float powerFR = gamepad1.left_trigger - gamepad1.right_trigger;
            float powerBR = gamepad1.left_trigger - gamepad1.right_trigger;
            float powerFL = gamepad1.left_trigger - gamepad1.right_trigger;
            float powerBL = gamepad1.left_trigger - gamepad1.right_trigger;

            float powerFR1 = + gamepad1.right_stick_x;
            float powerBR1 = - gamepad1.right_stick_x;
            float powerFL1 = - gamepad1.right_stick_x;
            float powerBL1 = + gamepad1.right_stick_x;

            float powerFR2 = + gamepad1.left_stick_x;
            float powerBR2 = + gamepad1.left_stick_x;
            float powerFL2 = - gamepad1.left_stick_x;
            float powerBL2 = - gamepad1.left_stick_x;

            FL.setPower(powerFL*VFL + powerFL1*VFL + powerFL2*VFL);
            BL.setPower(powerBL*VBL + powerBL1*VBL + powerBL2*VFR);
            FR.setPower(powerFR*VFR + powerFR1*VFR + powerFR2*VFR);
            BR.setPower(powerBR*VBR + powerBR1*VBR + powerBR2*VBR);

            temporaria++;

            if (temporaria>100) {

                if (gamepad2.dpad_up && gamepad2.right_bumper && VFL < 1) {
                    VFL += 0.1;
                } else if (gamepad2.dpad_up && gamepad2.left_bumper && VFL > 0) {
                    VFL -= 0.1;
                }

                if (gamepad2.dpad_up && gamepad2.right_bumper && VBL < 1) {
                    VBL += 0.1;
                } else if (gamepad2.dpad_up && gamepad2.left_bumper && VBL > 0) {
                    VBL -= 0.1;
                }

                if (gamepad2.dpad_up && gamepad2.right_bumper && VFR < 1) {
                    VFR += 0.1;
                } else if (gamepad2.dpad_up && gamepad2.left_bumper && VFR > 0) {
                    VFR -= 0.1;
                }

                if (gamepad2.dpad_up && gamepad2.right_bumper && VBR < 1) {
                    VBR += 0.1;
                } else if (gamepad2.dpad_up && gamepad2.left_bumper && VBR > 0) {
                    VBR -= 0.1;
                }

                temporaria=0;

            }

            telemetry.addData("VFL", VFL);
            telemetry.addData("VFR", VFR);
            telemetry.addData("VBL", VBL);
            telemetry.addData("VFR", VFR);
            telemetry.update();
        }
    }
}