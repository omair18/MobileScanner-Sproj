/*
 * Copyright 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.boofcv.android.camera2basic;

import android.Manifest;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Camera;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import org.boofcv.android.DemoMain;
import org.boofcv.android.R;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import static org.boofcv.android.DemoMain.preference;

public class CameraActivity extends AppCompatActivity {

    private String TAG = "STORAGE";
    private static final int MY_PERMISSIONS_REQUEST_CAMERA = 1;
    private static final int MY_PERMISSIONS_REQUEST_STORAGE = 2;

    public static String MainFolder = "/Scanner3D";
    public static String RectifiedFolder = MainFolder+"/Rectified";
    public static String PicturesFolder = MainFolder+"/Pictures";
    public static String DisparityFolder = MainFolder+"/Disparity";
    public static String PointCloudFolder = MainFolder+"/PointClouds";

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);
        checkAndRequestPermissions();
        createFolders();
        if(preference == null) {
            //ab.setMessage("Your camera needs to be Calibrated").set
            AlertDialog.Builder ab = new AlertDialog.Builder(this);
            ab.setMessage("Your Camera needs to be calibrated!")
                    .setCancelable(false)
                    .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                        public void onClick(DialogInterface dialog, int id) {
                            //do things
                            Intent intent = new Intent(CameraActivity.this, DemoMain.class);
                            startActivity(intent);
                        }
                    });
            AlertDialog alert = ab.create();
            alert.show();
        }
        else {
            Toast.makeText(CameraActivity.this, "Nice. Camera is already calibrated!", Toast.LENGTH_SHORT).show();
        }

        if (null == savedInstanceState) {
            getSupportFragmentManager().beginTransaction()
                    .replace(R.id.container, Camera2BasicFragment.newInstance())
                    .commit();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (preference == null) {
            //ab.setMessage("Your camera needs to be Calibrated").set
            AlertDialog.Builder ab = new AlertDialog.Builder(this);
            ab.setMessage("Your Camera needs to be calibrated!")
                    .setCancelable(false)
                    .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                        public void onClick(DialogInterface dialog, int id) {
                            //do things
                            Intent intent = new Intent(CameraActivity.this, DemoMain.class);
                            startActivity(intent);
                        }
                    });
            AlertDialog alert = ab.create();
            alert.show();
        }
        else {
            Toast.makeText(CameraActivity.this, "Nice. Camera is already calibrated!", Toast.LENGTH_SHORT).show();
            getSupportFragmentManager().beginTransaction()
                    .replace(R.id.container, Camera2BasicFragment.newInstance())
                    .commit();
        }
    }

    private void createFolders() {
        File mf = new File(Environment.getExternalStorageDirectory().toString()+MainFolder);
        if(!mf.exists())
            mf.mkdirs();
        File rf = new File(Environment.getExternalStorageDirectory().toString()+RectifiedFolder);
        if(!rf.exists())
            rf.mkdirs();
        File df = new File(Environment.getExternalStorageDirectory().toString()+DisparityFolder);
        if(!df.exists())
            df.mkdirs();
        File pf = new File(Environment.getExternalStorageDirectory().toString()+PointCloudFolder);
        if(!pf.exists())
            pf.mkdirs();
        File ppf = new File(Environment.getExternalStorageDirectory().toString()+PicturesFolder);
        if(!ppf.exists())
            ppf.mkdirs();
    }
    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_CAMERA:
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    //Permission Granted Successfully. Write working code here.
                } else {
                    //You did not accept the request can not use the functionality.
                }
                break;
            case MY_PERMISSIONS_REQUEST_STORAGE:
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    //Permission Granted Successfully. Write working code here.
                } else {
                    //You did not accept the request can not use the functionality.
                }
                break;
        }
    }
    private boolean checkAndRequestPermissions() {
        int permissionCAMERA = ContextCompat.checkSelfPermission(this,
                Manifest.permission.CAMERA);


        int storagePermission = ContextCompat.checkSelfPermission(this,


                Manifest.permission.WRITE_EXTERNAL_STORAGE);



        List<String> listPermissionsNeeded = new ArrayList<>();
        if (storagePermission != PackageManager.PERMISSION_GRANTED) {
            listPermissionsNeeded.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        }
        if (permissionCAMERA != PackageManager.PERMISSION_GRANTED) {
            listPermissionsNeeded.add(Manifest.permission.CAMERA);
        }
        if (!listPermissionsNeeded.isEmpty()) {
            ActivityCompat.requestPermissions(this,


                    listPermissionsNeeded.toArray(new String[listPermissionsNeeded.size()]), MY_PERMISSIONS_REQUEST_CAMERA);
            ActivityCompat.requestPermissions(this,


                    listPermissionsNeeded.toArray(new String[listPermissionsNeeded.size()]), MY_PERMISSIONS_REQUEST_STORAGE);
            return false;
        }

        return true;
    }
}
