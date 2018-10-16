package com.reactlibrary;

import com.facebook.react.bridge.Promise;
import com.facebook.react.bridge.ReactApplicationContext;
import com.facebook.react.bridge.ReactContextBaseJavaModule;
import com.facebook.react.bridge.ReactMethod;
import com.facebook.react.bridge.Callback;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.android.Utils;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import android.net.Uri;
import android.os.Environment;
import android.util.Base64;

import java.io.File;
import java.io.FileOutputStream;

public class RNOpenCvLibraryModule extends ReactContextBaseJavaModule {

    private final ReactApplicationContext reactContext;

    public RNOpenCvLibraryModule(ReactApplicationContext reactContext) {
        super(reactContext);
        this.reactContext = reactContext;
    }

    @Override
    public String getName() {
        return "RNOpenCvLibrary";
    }

    @ReactMethod
    public void checkForBlurryImage(String imageAsBase64, Callback errorCallback, Callback successCallback) {
        try {
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inDither = true;
            options.inPreferredConfig = Bitmap.Config.ARGB_8888;

            byte[] decodedString = Base64.decode(imageAsBase64, Base64.DEFAULT);
            Bitmap image = BitmapFactory.decodeByteArray(decodedString, 0, decodedString.length);


//      Bitmap image = decodeSampledBitmapFromFile(imageurl, 2000, 2000);
            int l = CvType.CV_8UC1; //8-bit grey scale image
            Mat matImage = new Mat();
            Utils.bitmapToMat(image, matImage);
            Mat matImageGrey = new Mat();
            Imgproc.cvtColor(matImage, matImageGrey, Imgproc.COLOR_BGR2GRAY);

            Bitmap destImage;
            destImage = Bitmap.createBitmap(image);
            Mat dst2 = new Mat();
            Utils.bitmapToMat(destImage, dst2);
            Mat laplacianImage = new Mat();
            dst2.convertTo(laplacianImage, l);
            Imgproc.Laplacian(matImageGrey, laplacianImage, CvType.CV_8U);
            Mat laplacianImage8bit = new Mat();
            laplacianImage.convertTo(laplacianImage8bit, l);

            Bitmap bmp = Bitmap.createBitmap(laplacianImage8bit.cols(), laplacianImage8bit.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(laplacianImage8bit, bmp);
            int[] pixels = new int[bmp.getHeight() * bmp.getWidth()];
            bmp.getPixels(pixels, 0, bmp.getWidth(), 0, 0, bmp.getWidth(), bmp.getHeight());
            int maxLap = -16777216; // 16m
            for (int pixel : pixels) {
                if (pixel > maxLap)
                    maxLap = pixel;
            }

//            int soglia = -6118750;
            int soglia = -8118750;
            if (maxLap <= soglia) {
                System.out.println("is blur image");
            }

            successCallback.invoke(maxLap <= soglia);
        } catch (Exception e) {
            errorCallback.invoke(e.getMessage());
        }
    }

    @ReactMethod
    public void rotateImage(String imagePath, Promise promise) {
        try {
            String path = Uri.parse(imagePath).getEncodedPath();
            Mat dataSrc = Imgcodecs.imread(path, Imgcodecs.IMREAD_COLOR);
            Bitmap imageSrc = BitmapFactory.decodeFile(path);
            Mat dataRot = new Mat();
            Mat dataSrc2 = new Mat();
            Imgproc.cvtColor(dataSrc,dataSrc2,Imgproc.COLOR_BGR2RGB);
            if (imageSrc == null)
                imageSrc = Bitmap.createBitmap(dataSrc.cols(), dataSrc.rows(), Bitmap.Config.ARGB_8888);
            Core.rotate(dataSrc2, dataRot, Core.ROTATE_90_CLOCKWISE);
            Bitmap imageOut = Bitmap.createBitmap(dataRot.cols(), dataRot.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(dataRot, imageOut, true);
            String file_path = Environment.getExternalStorageDirectory().getAbsolutePath() +
                    "/ImagesRotated";
            File dir = new File(file_path);
            if (!dir.exists())
                dir.mkdirs();
            String[] tmp = path.split("/");
            String name = tmp[tmp.length - 1].replace(".jpg", "");
            File file = new File(dir, name + "_Rotate.png");
            File file2 = new File(dir, name + "_Original.png");
            FileOutputStream fOut = new FileOutputStream(file);
            FileOutputStream fOut2 = new FileOutputStream(file2);
            imageOut.compress(Bitmap.CompressFormat.PNG, 100, fOut);
            imageSrc.compress(Bitmap.CompressFormat.PNG, 100, fOut2);
            fOut.flush();
            fOut.close();
            fOut2.flush();
            fOut2.close();
            promise.resolve(true);
            //successCallback.invoke("image rotated!!");
        } catch (Exception e) {
            promise.reject("100", e.getMessage());
        }
    }
}
