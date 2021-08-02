package com.antilatency.alttrackingdemo;

import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import android.text.method.ScrollingMovementMethod;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("AltTrackingSample");
        System.loadLibrary("AntilatencyDeviceNetwork");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView tv = findViewById(R.id.demo_text);
        tv.setMovementMethod(new ScrollingMovementMethod());

        Init();

        final Handler handler = new Handler();
        handler.post(new Runnable() {
            @Override
            public void run() {
                TextView tv = findViewById(R.id.demo_text);
                String text = tv.getText().toString() + GetOutput();

                String[] textArray = text.split("\n");
                String newText = "";
                int startIndex = 0;
                if (textArray.length > 200){
                    startIndex = textArray.length - 200;
                }

                for(int i = startIndex; i < textArray.length; ++i){
                    newText += textArray[i] + "\n";
                }

                tv.setText(newText);

                handler.postDelayed(this,500);
            }
        });
    }

    public native void Init();
    public native String GetOutput();
}