package jp.jaxa.iss.kibo.rpc.sampleapk;

        import android.graphics.Bitmap;
        import android.util.Log;
        import com.google.zxing.BinaryBitmap;
        import com.google.zxing.ChecksumException;
        import com.google.zxing.FormatException;
        import com.google.zxing.LuminanceSource;
        import com.google.zxing.NotFoundException;
        import com.google.zxing.RGBLuminanceSource;
        import com.google.zxing.common.HybridBinarizer;
        import com.google.zxing.qrcode.QRCodeReader;

        import org.jetbrains.annotations.Contract;
        import org.opencv.core.Mat;

        import java.util.Arrays;
        import java.util.regex.Matcher;
        import java.util.regex.Pattern;

        import gov.nasa.arc.astrobee.Result;
        import gov.nasa.arc.astrobee.types.Point;
        import gov.nasa.arc.astrobee.types.Quaternion;
        import gov.nasa.arc.astrobee.types.Point;
        import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
        import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        // astrobee is undocked and the mission starts
        long t_0 = System.currentTimeMillis()/1000;
        api.startMission();
        Mat cameraMatrix =AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Point pointA1 = new Point(10.3,-9.8,4.5);
        Quaternion quatA1 = new Quaternion(0,0,0,1);
        Point pointA2 = new Point(11.35,-10,4.5);
        Quaternion quatA2 = new Quaternion(0,0,-0.707f,0.707f);
        Point pointA3 = new Point(11.35,-10,4.95);
        Quaternion quatA3 = new Quaternion(0,0,-0.707f,0.707f);
        long t_1 = System.currentTimeMillis()/1000;
        Log.e("Time:start mission", String.valueOf(t_1 - t_0));
        moveToWrapper2(pointA1,quatA1);
        long t_2 = System.currentTimeMillis()/1000;
        Log.e("Time:move1", String.valueOf(t_2 - t_1));

        //pointAに二段階で移動
        moveToWrapper2(pointA2,quatA2);
        long t_3 = System.currentTimeMillis()/1000;
        Log.e("Time:move2", String.valueOf(t_3 - t_2));
        moveToWrapper2(pointA3,quatA3);
        long t_4 = System.currentTimeMillis()/1000;
        Log.e("Time:move3", String.valueOf(t_4 - t_3));

        Log.e("bmp1_1", "Start ZXing QR reading");
        String valueX = readQRcodeWrapper();
        Log.e("bmp1_1", "Finished ZXing QR reading. String valueX is[" + valueX + "]");
        api.sendDiscoveredQR(valueX);
        Log.e("bmp1_1", "QR code valueX has sent.");
        double[] pxyz = qrtopxyz(valueX);
        Log.e("pattern and pointAA", "[pattern, x, y, z]=" + Arrays.toString(pxyz));

        //QRから読み取った文字列から数値のみ抜き出してdoubleに変換
        int p = (int) pxyz[0];
        double aax = pxyz[1];
        double aay = pxyz[2];
        double aaz = pxyz[3];
        Log.e("pattern and pointAA x y z", "pattern:" + p + "[x:" + aax + "y:" + aay + "z:" + aaz + "]");
        long t_5 = System.currentTimeMillis()/1000;
        Log.e("Time:read QR and calculate A'", String.valueOf(t_5 - t_4));
        //回転クォータニオン
        //Quaternion rotate2AR = new Quaternion(0,(float) - Math.sin(Math.toRadians(30/2)),0, (float) Math.cos(Math.toRadians(30/2)));
        //Quaternion quatA4 = M.mul(quatA3, rotate2AR);
        //moveToWrapper2(pointA3,quatA4);


        //A'への移動経路
        pathplan(p,aax,aay,aaz,pointA3,quatA3);
        long t_6 = System.currentTimeMillis()/1000;
        Log.e("Time:pathplan", String.valueOf(t_6 - t_5));
/*
        //ARからターゲットまでの距離を考えて補正
        double[] stu = adjustment(p, aax, aaz);
        double axa = stu[0];
        double aya = aay - 0.2;
        double aza = stu[1];
        */
        //moveToWrapper(axa, aya, aza, quatA3.getX(), quatA3.getY(), quatA3.getZ(), quatA3.getW());

        //read ARcode

        Log.e("sleep 5 sec", "");
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Log.e("start read ARcode", "");
        //relativemoveToWrapper(0,0,0,0,0,-0.707f,0.707f);
        long t_7 = System.currentTimeMillis()/1000;
        Log.e("Time:sleep", String.valueOf(t_7 - t_6));

        pathplan3(p,aax,aay,aaz,pointA3,quatA3);
        //moveToWrapper(aax, aay, aaz, quatA3.getX(), quatA3.getY(), quatA3.getZ(), quatA3.getW());
        long t_8 = System.currentTimeMillis()/1000;
        Log.e("Time:stay A'", String.valueOf(t_8 - t_7));
        //pathplan(p,aax,aay,aaz,pointA3,quatA3);
        //moveToWrapper2(pointA3,quatA4);


        //ここからとりあえずコメントアウト
        //PointCloud point_cloud = api.getPointCloudHazCam();
        //double[][] array = point_cloud2list(point_cloud);

/*
        for (int i = 0; i < array.length; i++){
            Log.e("",String.valueOf(array[i][0]) + " " + String.valueOf(array[i][1]) + " " + String.valueOf(array[i][2]));
        }
*/

        //int i = 104;
        //Log.e("Hazcam",String.valueOf(array[i][0]) + " " + String.valueOf(array[i][1]) + " " + String.valueOf(array[i][2]));
        //double distance = - array[i][2] + aay - 0.1328;
        //Log.e("Target plane is located in", String.valueOf(distance));
        long t_9 = System.currentTimeMillis()/1000;
        Log.e("Time:HazCam", String.valueOf(t_9 - t_8));
        //moveToWrapper(aax, aay, aaz, quatA3.getX(), quatA3.getY(), quatA3.getZ(), quatA3.getW());
        pathplan3(p,aax,aay,aaz,pointA3,quatA3);
        long t_10 = System.currentTimeMillis()/1000;
        Log.e("Time:stay", String.valueOf(t_10 - t_9));




        //ターゲットの中心座標を４つのARの位置から求める
        double[] target_center = readARcode();
        //double[] target_center2 = readARcode_from_one();
        Log.e("finished reading ARcode", Arrays.toString(target_center));
        long t_11 = System.currentTimeMillis()/1000;
        ////Log.e("Time:read AR", String.valueOf(t_11 - t_10));
        //ターゲットの中心座標を１つのARの位置から求める
        //Log.e("finished reading ARcode2", Arrays.toString(target_center2));

        //opencvの座標系からastrobeeの座標系に変換
        target_center = change_origin(target_center);
        //double[] a = {0.15, 0, 0};

        //target_center = M.addVec(target_center, a);
        //target_center[0] = array[i][2] + 0.1328;
        Log.e("finished changing the origin", Arrays.toString(target_center));
        double[] navcam = {0.1177, -0.0422, -0.0826};
        double[] laser = {0.1302, 0.0572, -0.1111};
        double[] beam = {1, 0, 0};
        //レーザーをターゲット中心に向けるクォータニオンq_target
        Log.e("calculate q_target", "");
        Quaternion q_target = rotatetotarget(target_center, navcam, laser, beam);

        String del = ":";
        //astrobee座標系でのクォータニオンの計算終了
        Log.e("finished calculating q_target in astrobee's origin", String.valueOf(q_target.getX()) + del + String.valueOf(q_target.getY()) + del + String.valueOf(q_target.getZ()) + del + String.valueOf(q_target.getW()));
        q_target = M.mul(quatA3, q_target);
        Quaternion quatA4 = new Quaternion(q_target.getX(), q_target.getY(), q_target.getZ(), q_target.getW());
        //kibo座標系でのクォータニオンの計算終了
        Log.e("finished calculating q_target in kibo' origin", String.valueOf(q_target.getX()) + del + String.valueOf(q_target.getY()) + del + String.valueOf(q_target.getZ()) + del + String.valueOf(q_target.getW()));
        //moveToWrapper(axa, aya, aza, quatA3.getX(), quatA3.getY(), quatA3.getZ(), quatA3.getW());
        //moveToWrapper2(pointA3,quatA4);

        double[] navcam2laser = M.diffVec(laser, navcam);
        //relativemoveToWrapper(0,0,0, q_target.getX(), q_target.getY(), q_target.getZ(), q_target.getW());

        long t_12 = System.currentTimeMillis()/1000;
        Log.e("Time:calculate q_target", String.valueOf(t_12 - t_11));
        //moveToWrapper(aax,aay,aaz, q_target.getX(), q_target.getY(), q_target.getZ(), q_target.getW());
        pathplan3(p,aax,aay,aaz,pointA3,quatA4);
        Log.e("finished rotating", "");
        long t_13 = System.currentTimeMillis()/1000;

        Log.e("Time:rotate q_target", String.valueOf(t_13 - t_12));
        //Log.e("finish read ARcode", rotationMat.dump());
        //int corners = readARcode2();
        //Log.e("corners", String.valueOf(corners));
        /*
        String[][] EulerAngles0_s = new String[3][3];
        EulerAngles0_s[1][0] = String.valueOf(target_center[1][0]);
        Log.e("rotaionMatrix2", EulerAngles0_s[1][0]);
        //なんかおかしい
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                EulerAngles0_s[i][j] = String.valueOf(target_center[i][j]);
                Log.e("rotaionMatrix2", EulerAngles0_s[i][j]);
                Log.e("i", String.valueOf(i));
                Log.e("j", String.valueOf(j));
            }
        }


        String[] EulerAngles0_s2 = new String[EulerAngles0_s.length];
        for (int i = 0; i < EulerAngles0_s.length; i++) {
            EulerAngles0_s2[i] = TextUtils.join(",", EulerAngles0_s[i]);
            Log.e("rotaionMatrix2", EulerAngles0_s2[i]);
        }
        String EulerAngles0_s3 = TextUtils.join(",", EulerAngles0_s2);
        //moveToWrapper(aax, aay, aaz,0, 0, -0.7071068,0.7071068);
        //moveToA''
        Log.e("rotationMatrix_all", EulerAngles0_s3);
        */
        /*
        moveToWrapper(axa, aya, aza, 0, 0, -0.7071068, 0.7071068);
        Log.e("move to AA", "bee finished moving to pointAA.");
        */
        // irradiate the laser


/*
        Mat ARcode = readARcode2();
        String zero = ARcode.toString();
        Log.e("ARcode Image", zero);
        */
        Log.e("Laser", "Bee start irradiating the laser.");
        api.laserControl(true);
        long t_14 = System.currentTimeMillis()/1000;
        Log.e("Time:start irradiating the laser", String.valueOf(t_14 - t_13));

        // Take a snapshot
        Log.e("Snapshot", "Bee start taking a snapshot .");
        //moveToWrapper(aax,aay,aaz,q_target.getX(),q_target.getY(),q_target.getZ(),q_target.getW());
        pathplan3(p,aax,aay,aaz,pointA3,quatA4);
        api.takeSnapshot();
        Log.e("Snapshot", "Bee finished taking a snapshot .");

        long t_15 = System.currentTimeMillis()/1000;
        Log.e("Time:Snap shot", String.valueOf(t_15 - t_14));

        // turn off the laser
        api.laserControl(false);
        Log.e("Time:Laser", "Bee quit irradiating the laser.");
        long t_16 = System.currentTimeMillis()/1000;
        Log.e("Time:finish irradiating the laser", String.valueOf(t_16 - t_15));

        //Bへの移動
        pathplan2(p,aax,aay,aaz,pointA3,quatA3);
        long t_17 = System.currentTimeMillis()/1000;
        Log.e("Time:pathplan2", String.valueOf(t_17 - t_16));

        // Send mission completion
        api.reportMissionCompletion();
        Log.e("MissionCompletion", "Mission completed.");
        long t_18 = System.currentTimeMillis()/1000;
        Log.e("Time:Mission completed", String.valueOf(t_18 - t_17));
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);
        Log.e("Moveto","Bee start moving to x:"+pos_x+" y:"+pos_y+" z:"+pos_z);
        Log.e("Moveto","first challenge");
        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;

        //条件文&&に変更
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX)
        {
            Log.e("Moveto","Bee start moving to x:"+pos_x+" y:"+pos_y+" z:"+pos_z);
            Log.e("Moveto","loopCounter = "+loopCounter);
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private void moveToWrapper2(Point point,Quaternion quat) {
        final int LOOP_MAX = 3;
        Result result = api.moveTo(point, quat, true);
        Log.e("Moveto", "Bee start moving to x:" + point.getX()
                + " y:" + point.getY() + " z:" + point.getZ());
        Log.e("Moveto","first challenge");
        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            Log.e("Moveto", "Bee start moving to x:" + point.getX()
                    + " y:" + point.getY() + " z:" + point.getZ());
            Log.e("Loop Counter", "loop counter is" + loopCounter);

            result = api.moveTo(point, quat, true);
            ++loopCounter;
        }
    }

    private void relativemoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {
        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.relativeMoveTo(point, quaternion, true);

        int loopCounter = 0;
        while (result.hasSucceeded() && loopCounter < LOOP_MAX) {
            Log.e("Moveto", "Bee start moving to x:" + pos_x + " y:" + pos_y + " z:" + pos_z);
            Log.e("Loop Counter relative", "loop counter is" + loopCounter);

            result = api.relativeMoveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public double[] change_origin(double[] point) {
        double[] changed_point = new double[3];
        changed_point[0] = point[2];
        changed_point[1] = point[0];
        changed_point[2] = point[1];
        return changed_point;
    }

/*
    public double[] change_origin(double[] point) {
        double[] changed_point = new double[3];
        changed_point[0] = point[0];
        changed_point[1] = -point[2];
        changed_point[2] = point[1];
        return changed_point;
    }
    */

    public String readQrcode(Bitmap bitmap) {
        //final int navcamWidth = 1280;
        //final int navcamHeight = 960;
        final int trimStartx = 480;
        final int trimStarty = 500;
        final int trimWidth = 320;
        final int trimHeight = 380;

        //トリミング作業
        Bitmap bitmap_trim = Bitmap.createBitmap(bitmap, trimStartx, trimStarty, trimWidth, trimHeight);
        // Bitmap のサイズを取得して、ピクセルデータを取得する
        int width = bitmap_trim.getWidth();
        Log.d("readQR", "bitmap width=" + width);
        int height = bitmap_trim.getHeight();
        Log.d("readQR", "bitmap height=" + height);
        int[] pixels = new int[width * height];
        Log.e("bmp1_1", "getPixels start.");
        bitmap_trim.getPixels(pixels, 0, width, 0, 0, width, height);
        Log.e("bmp1_1", "getPixels finished.");

        try {
            // zxing で扱える BinaryBitmap形式に変換する
            Log.e("bmp1_1", "RGB start.");
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            Log.e("bmp1_1", "RGB finished.");
            //Log.e("bmp1_1","binarybitmap start.");
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            //Log.e("bmp1_1","binarybitmap finished.");

            // zxing で画像データを読み込み解析する
            //Log.e("bmp1_1","QRcodereader start.");
            QRCodeReader reader = new QRCodeReader();
            //Log.e("bmp1_1","QRcodereader finished.");
            Log.e("bmp1_1", "decord start.");
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap);
            Log.e("bmp1_1", "decord finished.");

            // 解析結果を取得する
            //Log.e("bmp1_1","gettext start.");
            String result = decodeResult.getText();
            Log.d("readQR", result);
            return result;
        }
        //エラーの原因出力
        catch (NotFoundException e) {
            //QRが見つからなかったとき
            Log.e("readQR", "NotFoundException occur : " + e.getMessage());
            return null;
        } catch (FormatException e) {
            Log.e("readQR", "CheckSumException occur : " + e.getMessage());
            return null;
        } catch (ChecksumException e) {
            Log.e("readQR", "FormatException occur : " + e.getMessage());
            return null;
        }
    }

    private String readQRcodeWrapper() {
        final int LOOP_MAX = 3;
        int loopCounter = 0;
        String result = null;
        Bitmap bitmap;

        while (result == null && loopCounter < LOOP_MAX) {
            Log.e("readQRtry", "loopCounter = " + loopCounter);
            bitmap = api.getBitmapNavCam();
            result = readQrcode(bitmap);
            ++loopCounter;
        }
        return result;
    }

    public static double[] qrtopxyz(String qr) {
        String cutstring = "[{]|[}]|[\"p\":]|[\"x\":]|[\"y\":]|[\"z\":]";
        Pattern cutpattern = Pattern.compile(cutstring);
        Matcher m = cutpattern.matcher(qr);
        qr = m.replaceAll("");
        String[] stringpxyz = qr.split(",");
        double[] pxyz = new double[4];
        for (int i = 0; i < 4; i++) {
            pxyz[i] = Double.parseDouble(stringpxyz[i]);
        }
        return pxyz;
    }

    //@org.jetbrains.annotations.Contract(pure = true)
    @Contract("null -> fail; !null -> !null")
    public static double[] adjustment(int num1, double num2, double num4) {
        double[] stu = new double[2];
        if ((num1 == 1) || (num1 == 8)) {
            double num22 = num2 - 0.1125;
            //double aya = num3;
            double num44 = num4 + 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        } else if ((num1 == 2) || (num1 == 3) || (num1 == 4)) {
            double num22 = num2 + 0.1125;
            //double aya = num3;
            double num44 = num4 + 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        } else if ((num1 == 5) || (num1 == 6)) {
            double num22 = num2 + 0.1125;
            //double aya = num3;
            double num44 = num4 - 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        } else {
            double num22 = num2 - 0.1125;
            //double aya = num3;
            double num44 = num4 - 0.0415;
            stu[0] = num22;
            stu[1] = num44;
        }
        return stu;
    }
    
    //A'への経路パターン別
    //adx,ady,adzがA’の座標pxyz[]
    private void pathplan(int p, double adx, double ady, double adz, Point pointA,Quaternion quatA) {
        Point pointAd = new Point(adx,ady,adz);
        if ((p == 1) || (p == 2) ||(p == 8)) {
            ;
            //moveToWrapper2(pointAd,quatA);
            //Log.e("finished moving 1", "");
        } else if ((p == 3) || (p == 4)) {
            //2段階移動(1回目：xyのみ移動，2回目：zのみ移動)
            Point pointAA = new Point(adx,ady,pointA.getZ());
            moveToWrapper2(pointAA,quatA);
            Log.e("finished moving 1", "");
            //moveToWrapper2(pointAd,quatA);
            //Log.e("finished moving 2", "");
        } else if ((p == 5) || (p == 6)) {
            //Point pointAA1 = new Point(10.8,ady,pointA.getZ());
            Point pointAA1 = new Point(10.6,ady,pointA.getZ());
            //Point pointAA2 = new Point(10.8,ady,adz);
            Point pointAA2 = new Point(10.6,ady,adz);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointAA2,quatA);
            Log.e("finished moving 2", "");
            //moveToWrapper2(pointAd,quatA);
            //Log.e("finished moving 3", "");
        } else {
            Point pointAA1 = new Point(11.5,ady,pointA.getZ());
            Point pointAA2 = new Point(11.5,ady,adz);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointAA2,quatA);
            Log.e("finished moving 2", "");
            //moveToWrapper2(pointAd,quatA);
            //Log.e("finished moving 3", "");
        }
    }

    //snapshotとgetnavcam用
    private void pathplan3(int p,double adx, double ady, double adz,Point pointA,Quaternion quatA) {
        Point pointAd = new Point(adx,ady,adz);
        Point pointA3 = new Point(11.35,-10,4.95);
        Quaternion quatA3 = new Quaternion(0,0,-0.707f,0.707f);
        if ((p == 1) || (p == 2) ||(p == 8)) {
            moveToWrapper2(pointA3,quatA);
            Log.e("finished moving 1", "");
        } else if ((p == 3) || (p == 4)) {
            //2段階移動(1回目：xyのみ移動，2回目：zのみ移動)
            Point pointAA = new Point(adx,ady,pointA.getZ());
            moveToWrapper2(pointAA,quatA);
            Log.e("finished moving 1", "");
            //moveToWrapper2(pointAd,quatA);
            //Log.e("finished moving 2", "");
        } else if ((p == 5) || (p == 6)) {
            Point pointAA2 = new Point(10.6,ady,adz);
            moveToWrapper2(pointAA2,quatA);
            Log.e("finished moving 2", "");
        } else {
            Point pointAA2 = new Point(11.5,ady,adz);
            moveToWrapper2(pointAA2,quatA);
            Log.e("finished moving 2", "");
        }
    }

    public double[] readARcode() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        double[] target_center = AR.detectTarget(mat3, cameraMatrix, distortionCoefficients);
        return target_center;
    }

    public double[] readARcode_from_one() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        double[] target_center = AR.detectTarget_from_one(mat3, cameraMatrix, distortionCoefficients);
        return target_center;
    }

    public double[] readARcode_from_one2() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        double[] target_center = AR.detectTarget_from_one2(mat3, cameraMatrix, distortionCoefficients);
        return target_center;
    }


    public String readARcode3() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        String target_center = AR.detectTarget2(mat3, cameraMatrix, distortionCoefficients);
        return target_center;
    }

    public void readARcode4() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        AR.detectTarget3(mat3, cameraMatrix, distortionCoefficients);
    }


    public int readARcode2() {
        Mat cameraMatrix = AR.makeCamMat();
        Mat distortionCoefficients = AR.makeDistCoef();
        Mat mat3 = api.getMatNavCam();
        int corners = AR.MarkerImage(mat3, cameraMatrix, distortionCoefficients);
        return corners;
/*
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        IntentResult result = IntentIntegrator.parseActivityResult(requestCode, resultCode, data);
        if(result != null) {
            Log.d("readQR", result.getContents());
        } else {
            super.onActivityResult(requestCode, resultCode, data);
        }
    }

 */
    }

    public double[][] point_cloud2list(PointCloud point_cloud){
        Point[] PointArray = point_cloud.getPointArray();
        int width = point_cloud.getWidth();
        Log.e("pointcloud width", String.valueOf(width));
        int height = point_cloud.getHeight();
        Log.e("pointcloud height", String.valueOf(height));
        int length = width * height;
        double[][] array = new double[length][3];
        for(int i = 0; i < length; i++) {
            array[i][0] = PointArray[i].getX();
            array[i][1] = PointArray[i].getY();
            array[i][2] = PointArray[i].getZ();
        }
        return array;
    }

    public Quaternion rotatetotarget(double[] target_center, double[] navcam, double[] laser, double[] beam) {
        //ビームの長さk
        //opencv座標の原点はカメラの位置なのか？
        target_center = M.addVec(target_center, navcam);
        double k =  Math.sqrt(Math.pow(target_center[0],2) + Math.pow(target_center[1],2) + Math.pow(target_center[2],2)
                - Math.pow(laser[1], 2) - Math.pow(laser[2], 2)) - laser[0];
        Log.e("k", String.valueOf(k));
        //回転前のビームの先端．ターゲット中心ベクトルと大きさ同じ
        double[] target_center_beam = M.addVec(laser, M.scalMul(beam, k));
        Log.e("target_center_beam", Arrays.toString(target_center_beam));
        Log.e("target_center", Arrays.toString(target_center));
        Quaternion q_target = M.diffv2v_2(target_center, target_center_beam);
        return q_target;
    }

    //Bへの経路パターン別1,3,4,5,6
    //adx,ady,adzがA’の座標pxyz[]
    private void pathplan2(int p, double adx, double ady, double adz, Point pointA,Quaternion quatA) {
        Point pointAd = new Point(adx,ady,adz);
        Point pointB = new Point(10.6,-8.0,4.5);
        if ((p == 1)||(p == 8)){
            Point pointAA1 = new Point(adx-0.35,-9.0,adz-0.5);
            Point pointAA2 = new Point(pointAA1.getX()-0.35,-9.0,pointAA1.getZ()-0.1);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointAA2,quatA);
            Log.e("finished moving 2", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 3", "");
        }else if (p == 2){
            Point pointAA1 = new Point(adx-0.5,ady+0.8,adz);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 2", "");
        }else if (p == 3){
            Point pointAA1 = new Point(adx-0.5,ady+0.8,adz-0.08);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 2", "");
        }else if (p == 4){
            Point pointAA1 = new Point(adx-0.5,ady+0.8,adz-0.7);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 2", "");
        }else if (p == 5) {
            Point pointAA1 = new Point(10.6,-8.8,5.1);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 2", "");
        }else if (p == 6) {
            Point pointAA1 = new Point(10.6,-8.8,5.25);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 2", "");
        }else{
            Point pointAA1 = new Point(11.5,ady,adz);
            Point pointAA2 = new Point(pointAA1.getX(),ady,pointA.getZ());
            //Point pointAA3 = new Point(pointAA2.getX()-0.9,pointAA2.getY()+0.8,4.73);
            Point pointAA3 = new Point(10.6,-8.8,4.73);
            moveToWrapper2(pointAA1,quatA);
            Log.e("finished moving 1", "");
            moveToWrapper2(pointAA2,quatA);
            Log.e("finished moving 2", "");
            moveToWrapper2(pointAA3,quatA);
            Log.e("finished moving 3", "");
            moveToWrapper2(pointB,quatA);
            Log.e("finished moving 4", "");
        }
    }
}
