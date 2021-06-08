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
import org.opencv.core.Size;
import org.opencv.aruco.DetectorParameters;
import org.opencv.utils.Converters;


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


    public static double[] detectTarget(Mat inputImage, Mat cameraMatrix, Mat distortionCoefficients) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);

        Mat rotationMatrix = new Mat(), translationVectors = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distortionCoefficients, rotationMatrix, translationVectors);

        String del = ":";
        String value = rotationMatrix.dump() + del + translationVectors.dump();

        //List<Double> trans_vectors = new List<Double>();
        //Converters.Mat_to_vector_double(translationVectors, trans_vectors);
        Log.e("rotation vectors and translation vectors", value);
        double AR1[] = translationVectors.get(0,0);
        double AR2[] = translationVectors.get(1,0);
        double AR3[] = translationVectors.get(2,0);
        double AR4[] = translationVectors.get(3,0);
        double[] target_center = M.scalDiv(M.addVec(AR1, M.addVec(AR2, M.addVec(AR3, AR4))), 4);
        return target_center;
    }


    public static String detectTarget2(Mat inputImage, Mat cameraMatrix, Mat distortionCoefficients) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);
        Mat rotationMatrix = new Mat(), translationVectors = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distortionCoefficients, rotationMatrix, translationVectors);
        /*
        String del = ":";
        String value = rotationMatrix.dump() + del + translationVectors.dump() + del + cameraMatrix.dump();
        */
        Mat AR1 = translationVectors.row(0);
        double AR2[] = translationVectors.get(0,1);
        double AR3[] = translationVectors.get(0,2);
        double AR4[] = translationVectors.get(0,3);
        //double[] target_center = M.scalDiv(M.addVec(AR1, M.addVec(AR2, M.addVec(AR3, AR4))), 4);
        return translationVectors.dump();
    }

    public static void detectTarget3(Mat inputImage, Mat cameraMatrix, Mat distortionCoefficients) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);
        Mat rotationMatrix = new Mat(), translationVectors = new Mat();
        Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distortionCoefficients, rotationMatrix, translationVectors);
        /*
        String del = ":";
        String value = rotationMatrix.dump() + del + translationVectors.dump() + del + cameraMatrix.dump();
        */
        Size sizeM = translationVectors.size();
        for (int i = 0; i < sizeM.height; i++)
            for (int j = 0; j < sizeM.width; j++) {
                double[] data1 = translationVectors.get(i, j);
                Log.e("detectTarget3", String.valueOf(i)+":"+String.valueOf(j)+":"+Arrays.toString(data1));
            }
        ;
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