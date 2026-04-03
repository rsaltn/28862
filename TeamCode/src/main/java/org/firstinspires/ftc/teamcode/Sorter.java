package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

@Configurable
public class Sorter {
    CRServo servoRR;

    Servo servoL, servoRF;
    NormalizedColorSensor colorL, colorRF1, colorRF2, colorRR1, colorRR2;

    public SlotL slotL;
    public SlotRF slotRF;
    public SlotRR slotRR;
    public static double dl1= 0.02,drr2= 0.05,drf2= 0.01;

    public static double servoTransferPos = 0.8, servoStartPos = 0.05, transferTime = 0.3,rl = 0.05,gl = 0.12 ,bl = 0,rF = 0.03,gF = 0.07,bF=0,rR = 0.03,gR = 0.05,bR = 0.0;
    public static int GAIN = 10;

    public enum detectedColor {
        PURPLE,
        GREEN,
        NOTHING
    }

    public enum SlotId { L, RF, RR, NONE }

    public boolean transferActive = false;
    public detectedColor desiredColor = detectedColor.GREEN;
    private boolean fallbackUsed = false;
    public SlotId activeSlot = SlotId.NONE;

    public void init(HardwareMap hardwareMap) {
        servoL = hardwareMap.get(Servo.class, "servo_left");
        servoRF = hardwareMap.get(Servo.class, "servo_right_front");
        servoRR = hardwareMap.get(CRServo.class, "servo_right_rear");


        servoRF.setDirection(Servo.Direction.REVERSE);
        servoL.setDirection(Servo.Direction.REVERSE);
        colorL = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        colorRF1 = hardwareMap.get(NormalizedColorSensor.class, "color_right_front1");
        colorRF2 = hardwareMap.get(NormalizedColorSensor.class, "color_right_front2");
        colorRR1 = hardwareMap.get(NormalizedColorSensor.class, "color_right_rear1");
        colorRR2 = hardwareMap.get(NormalizedColorSensor.class, "color_right_rear2");

        colorL.setGain(GAIN);
        colorRF1.setGain(GAIN);



        colorRR2.setGain(GAIN);

        slotL = new SlotL();
        slotRF = new SlotRF();
        slotRR = new SlotRR();

        // стартовые позиции (на всякий случай)
        servoL.setPosition(0.385);
        servoRF.setPosition(0.06);
        servoRR.setPower(-0.016);
    }

    public void startTransfer(detectedColor color) {
        desiredColor = color;
        transferActive = true;
        fallbackUsed = false;
        activeSlot = SlotId.NONE;

        slotL.resetState();
        slotRF.resetState();
        slotRR.resetState();
    }

    public boolean updateTransfer() {
        if (!transferActive) return false;

        if (activeSlot == SlotId.NONE) {
            SlotId chosen = findSlotWithColor(desiredColor);

            if (chosen == SlotId.NONE && !fallbackUsed) {
                fallbackUsed = true;
                detectedColor fallbackColor = (desiredColor == detectedColor.GREEN)
                        ? detectedColor.PURPLE
                        : detectedColor.GREEN;

                chosen = findSlotWithColor(fallbackColor);
                if (chosen != SlotId.NONE) desiredColor = fallbackColor;
            }

            if (chosen == SlotId.NONE) {
                transferActive = false;
                return false;
            }

            activeSlot = chosen;
        }

        boolean finished = false;
        switch (activeSlot) {
            case RR: finished = slotRR.transferBall(true); break;
            case L:  finished = slotL.transferBall(true);  break;
            case RF: finished = slotRF.transferBall(true); break;
            default: break;
        }

        if (finished) {
            transferActive = false;
            activeSlot = SlotId.NONE;
            return false;
        }

        return true;
    }

    private SlotId findSlotWithColor(detectedColor color) {
        if (slotRF.getBallColor() == color) return SlotId.RF;
        if (slotRR.getBallColor() == color) return SlotId.RR;
        if (slotL.getBallColor()  == color) return SlotId.L;
        return SlotId.NONE;
    }

    public class SlotL {
        public boolean transferring = false, transferred = false;
        Timer transferTimer = new Timer();

        public void resetState() {
            transferring = false;
            transferred = false;
            transferTimer.resetTimer();
        }

        public detectedColor getBallColor() {
            NormalizedRGBA colors = colorL.getNormalizedColors();
            double a = colors.alpha;
            if (a < 1e-6) return detectedColor.NOTHING;

            double r = colors.red / a;
            double g = colors.green / a;
            double b = colors.blue / a;
            double max = Math.max(Math.max(r, g), b);

            if (!(r > rl && g > gl && b > bl)) {
                if (g == max) return detectedColor.GREEN;
                if (b == max) return detectedColor.PURPLE;
            }
            return detectedColor.NOTHING;
        }

        private void setServoToTransferPos() {
            if (!transferring) {
                servoL.setPosition(0.8);
                transferring = true;
                transferTimer.resetTimer();
            }
        }

        private void setServoToStartPos() {
            servoL.setPosition(0.385);
            transferred = true;
        }

        public boolean transferBall(boolean transfer) {
            if (!transfer) return false;

            transferred = false;
            setServoToTransferPos();

            if (transferring && (transferTimer.getElapsedTimeSeconds() > transferTime)) {
                transferring = false;
                setServoToStartPos();
                transferTimer.resetTimer();
            }
            return transferred;
        }
    }

    public class SlotRF {
        public boolean transferring = false, transferred = false;
        Timer transferTimer = new Timer();

        public void resetState() {
            transferring = false;
            transferred = false;
            transferTimer.resetTimer();
        }

        public detectedColor getBallColor() {
            NormalizedRGBA c1 = colorRF1.getNormalizedColors();
            NormalizedRGBA c2 = colorRF2.getNormalizedColors();

            double a = (c1.alpha + c2.alpha) / 2.0;
            if (a < 1e-6) return detectedColor.NOTHING;

            double r = ((c1.red / a) + (c2.red / a)) / 2.0;
            double g = ((c1.green / a) + (c2.green / a)) / 2.0;
            double b = ((c1.blue / a) + (c2.blue / a)) / 2.0;

            double max = Math.max(Math.max(r, g), b);

            if (!(r > rF && g > gF && b > bF)) {
                if (g == max) return detectedColor.GREEN;
                if (b == max) return detectedColor.PURPLE;
            }
            return detectedColor.NOTHING;
        }

        private void setServoToTransferPos() {
            if (!transferring) {
                servoRF.setPosition(0.6);
                transferring = true;
                transferTimer.resetTimer();
            }
        }

        private void setServoToStartPos() {
            servoRF.setPosition(0.06);
            transferred = true;
        }

        public boolean transferBall(boolean transfer) {
            if (!transfer) return false;

            transferred = false;
            setServoToTransferPos();

            if (transferring && (transferTimer.getElapsedTimeSeconds() > transferTime)) {
                transferring = false;
                setServoToStartPos();
                transferTimer.resetTimer();
            }
            return transferred;
        }
    }

    public class SlotRR {
        public boolean transferring = false, transferred = false;
        Timer transferTimer = new Timer();

        public void resetState() {
            transferring = false;
            transferred = false;
            transferTimer.resetTimer();
        }

        public detectedColor getBallColor() {
            NormalizedRGBA c1 = colorRR1.getNormalizedColors();
            NormalizedRGBA c2 = colorRR2.getNormalizedColors();

            double a = (c1.alpha + c2.alpha) / 2.0;
            if (a < 1e-6) return detectedColor.NOTHING;

            double r = ((c1.red / a) + (c2.red / a)) / 2.0;
            double g = ((c1.green / a) + (c2.green / a)) / 2.0;
            double b = ((c1.blue / a) + (c2.blue / a)) / 2.0;

            double max = Math.max(Math.max(r, g), b);

            if (!(r > rR && g > gR && b > bR)) {
                if (g == max) return detectedColor.GREEN;
                if (b == max) return detectedColor.PURPLE;
            }
            return detectedColor.NOTHING;
        }

        private void setServoToTransferPos() {
            if (!transferring) {
                servoRR.setPower(servoTransferPos-0.1);
                transferring = true;
                transferTimer.resetTimer();
            }
        }

        private void setServoToStartPos() {
            servoRR.setPower(-0.016);
            transferred = true;
        }

        public boolean transferBall(boolean transfer) {
            if (!transfer) return false;

            transferred = false;
            setServoToTransferPos();

            if (transferring && (transferTimer.getElapsedTimeSeconds() > transferTime)) {
                transferring = false;
                setServoToStartPos();
                transferTimer.resetTimer();
            }
            return transferred;
        }
    }
}
