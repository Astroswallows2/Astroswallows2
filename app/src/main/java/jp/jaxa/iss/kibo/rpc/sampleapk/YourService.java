package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;

import java.util.Arrays;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        // astrobee is undocked and the mission starts
        api.startMission();

        // astrobee is undocked and the mission starts
        //moveToWrapper(11.71, -9.53, 5.35, 0, 0, 0, 1);
        moveToWrapper(10.3, -9.8, 4.5, 0, 0, 0, 1);
        //api.getTrustedRobotKinematics();

        //pointAに移動
        moveToWrapper(11.21, -10, 5, 0, 0, -1 / Math.sqrt(2), 1 / Math.sqrt(2));
        //api.getTrustedRobotKinematics();

        /*追加分*/
        Bitmap bmp1_1 = api.getBitmapNavCam();
        Log.e("bmp1_1","has loaded as BitmapDockCam. Bitmap data bmp1_1 is["+bmp1_1);
        Log.e("bmp1_1","Start ZXing QR reading");
        String valueX = readQrcode(bmp1_1);
        Log.e("bmp1_1","Finished ZXing QR reading. String valueX is["+valueX+"]");
        api.sendDiscoveredQR(valueX);
        Log.e("bmp1_1","QR code valueX has sent.");
        double[] pxyz = qrtopxyz(valueX);
        Log.e("pattern and pointAA","[pattern, x, y, z]="+ Arrays.toString(pxyz));
        int p = (int)pxyz[0];
        double aax = pxyz[1];
        double aay = pxyz[2];
        double aaz = pxyz[3];
        Log.e("pattern and pointAA x y z","pattern:"+String.valueOf(p)+
                "[x:"+String.valueOf(aax)+ "y:"+String.valueOf(aay)+ "z:"+String.valueOf(aaz)+"]");
        moveToWrapper(aax, aay, aaz,0, 0, -0.7071068,0.7071068);
        Log.e("move to AA","bee finished moving to pointAA.");

        // irradiate the laser]
        Log.e("Laser","Bee start irradiating the laser.");
        api.laserControl(true);
        Log.e("Laser","Bee finished irradiating the laser.");
        // Take a snapshot
        Log.e("Snapshot","Bee start taking a snapshot .");
        api.takeSnapshot();
        Log.e("Snapshot","Bee finished taking a snapshot .");
        // Send mission completion
        api.reportMissionCompletion();
        Log.e("MissionCompletion","Mission completed.");
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

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            Log.e("Moveto","Bee start moving.");
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public String readQrcode(Bitmap bitmap)
    {
        // Bitmap のサイズを取得して、ピクセルデータを取得する
        int width = bitmap.getWidth();
        Log.d("readQR", "bitmap width="+width);
        int height = bitmap.getHeight();
        Log.d("readQR", "bitmap height="+height);
        int[] pixels = new int[width * height];
        Log.e("bmp1_1","getPixels start.");
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
        Log.e("bmp1_1","getPixels finished.");
        try {
            // zxing で扱える BinaryBitmap形式に変換する
            Log.e("bmp1_1","RGB start.");
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            Log.e("bmp1_1","RGB finished.");
            Log.e("bmp1_1","binarybitmap start.");
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            Log.e("bmp1_1","binarybitmap finished.");
            // zxing で画像データを読み込み解析する
            Log.e("bmp1_1","multiformatreader start.");
            Reader reader = new MultiFormatReader();
            Log.e("bmp1_1","multiformatreader finished.");
            Log.e("bmp1_1","decord start.");
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap);
            Log.e("bmp1_1","decord finished.");
            // 解析結果を取得する
            Log.e("bmp1_1","gettext start.");
            String result = decodeResult.getText();
            Log.d("readQR", result);
            return result;
            } catch (Exception e) {
                    Log.d("readQR", e.getLocalizedMessage());
                    return null;
            }
    }

    public static double[] qrtopxyz(String qr)
    {
        String cutstring = "[{]|[}]|[\"p\":]|[\"x\":]|[\"y\":]|[\"z\":]";
        Pattern cutpattern = Pattern.compile(cutstring);
        Matcher m = cutpattern.matcher(qr);
        qr = m.replaceAll("");
        String[] stringpxyz = qr.split(",");
        double[] pxyz = new double[4];
        for (int i = 0; i < 4; i++)
        {
            pxyz[i] = Double.parseDouble(stringpxyz[i]);
        }
        return pxyz;
    }
}
