package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Configurable
public class Camera {

    public Limelight3A camera;

    public static int aprilTagPipelineIndex = 0;

    public boolean hasTag = false;
    public double tx = 0.0;     // градусы (вправо/влево)
    public double ty = 0.0;
    public double ta = 0.0;
    public int seenId = -1;

    public void init(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.setPollRateHz(100);
        camera.start();

        camera.pipelineSwitch(aprilTagPipelineIndex);
    }

    public void update() {
        hasTag = false;
        seenId = -1;
        tx = 0; ty = 0; ta = 0;

        LLResult result = camera.getLatestResult();
        if (result == null || !result.isValid()) return;

        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            hasTag = true;
            seenId = fiducials.get(0).getFiducialId();
        }
    }
}
