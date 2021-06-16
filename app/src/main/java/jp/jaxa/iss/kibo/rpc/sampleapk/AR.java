package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.aruco.DetectorParameters;


import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.types.Point;

public class AR{

    public static Mat makeCamMat(){
        double camMatData[] = {344.173397, 0.000000, 630.793795, 0.000000, 344.277922, 487.033834, 0.000000, 0.000000, 1.000000};
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0,0,camMatData);
        return cameraMatrix;
        //返り値はカメラ行列
    }

    public static Mat makeDistCoef(){
        double distCoefData[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
        Mat distortionCoefficients = new Mat(1, 5, CvType.CV_64F);
        distortionCoefficients.put(0,0,distCoefData);
        return distortionCoefficients;
        //歪み行列
    }

    public static Mat detectMarker(Mat inputImage,Mat cameraMatrix,Mat distortionCoefficients) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

//        Mat inputImage = Imgcodecs.imread("F:\\users\\smk7758\\Desktop\\marker_2018-12-01_test.png");
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);

        Aruco.drawDetectedMarkers(inputImage, corners, markerIds);

        Mat rotationMatrix = new Mat(), translationVectors = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distortionCoefficients, rotationMatrix, translationVectors);
        String del = ":";
        String value = rotationMatrix.dump() + del + translationVectors.dump() + del + cameraMatrix.dump();




        Mat EulerAngles0 = rotationMatrix.row(0);

        return EulerAngles0;
    }

    public static int MarkerImage(Mat inputImage,Mat cameraMatrix,Mat distortionCoefficients) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);

//        Mat inputImage = Imgcodecs.imread("F:\\users\\smk7758\\Desktop\\marker_2018-12-01_test.png");
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);

        Aruco.drawDetectedMarkers(inputImage, corners, markerIds);

        Mat rotationMatrix = new Mat(), translationVectors = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distortionCoefficients,rotationMatrix, translationVectors);
/*
        for (int i = 0; i < markerIds.size().height; i++) { // TODO
            Aruco.drawAxis(inputImage, cameraMatrix, distortionCoefficients, rotationMatrix, translationVectors, 0.1f);
        }
        */

        return corners.size();
    }




}