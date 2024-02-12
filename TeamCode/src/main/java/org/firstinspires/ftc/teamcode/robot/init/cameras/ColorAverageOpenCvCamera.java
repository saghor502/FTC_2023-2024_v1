package org.firstinspires.ftc.teamcode.robot.init.cameras;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.init.cameras.pipelines.SplitAverageColorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ColorAverageOpenCvCamera {
    OpenCvCamera camera;
    SplitAverageColorPipeline splitAveragePipeline;
    int camW = 1280;
    int camH = 720;

    int zone = 3;

    public ColorAverageOpenCvCamera(HardwareMap hardwareMap, String cameraName){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName));
        splitAveragePipeline = new SplitAverageColorPipeline();

        camera.setPipeline(splitAveragePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void stopCameraStream(){
        camera.stopStreaming();
    }

    public void setAlliance(String alliance){
        splitAveragePipeline.setAlliancePipe(alliance);
    }

    public int elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public void toggleAverageZone(){
        splitAveragePipeline.toggleAverageZonePipe();
    }

    public double getMaxDistance(){
        return splitAveragePipeline.getMaxDistance();
    }
}

