package com.example.robotcontroller;

import androidx.appcompat.app.AppCompatActivity;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageButton;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends AppCompatActivity {

    private BluetoothService _bluetoothSerivce;
    private char _currentSignal;

    @SuppressLint("ClickableViewAccessibility")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        List<ImageButton> Buttons = new ArrayList<>();

        _bluetoothSerivce = new BluetoothService(this);

        Buttons.add((ImageButton) findViewById(R.id.Left));
        Buttons.add((ImageButton) findViewById(R.id.Up));
        Buttons.add((ImageButton) findViewById(R.id.Down));
        Buttons.add((ImageButton) findViewById(R.id.Right));

        for (ImageButton button : Buttons)
        {
            button.setOnTouchListener(new View.OnTouchListener() {

                @Override
                public boolean onTouch(View v, MotionEvent event) {
                    //Get ImageButton
                    ImageButton imgBtn = (ImageButton) v;
                    //Set opacity to 100%
                    imgBtn.setAlpha((float) 1.0);

                    //Get Id
                    String id = getResources().getResourceName(v.getId());
                    int idLocation = id.indexOf('/');
                    id = id.substring(idLocation +1,idLocation+2);

                    //Send signal containing first letter of direction
                    SendSignal(id.charAt(0));

                    //Finger up
                    if(event.getAction() == MotionEvent.ACTION_UP) {
                        //Set button opacity to 50%
                        imgBtn.setAlpha((float) 0.5);
                        //Send stop signal
                        SendSignal('S');
                    }

                    return true;
                }

            });

        }

    }

    //Sends a bluetooth signal
    public void SendSignal (char Direction){
        if(_currentSignal==Direction)
            return;

        _bluetoothSerivce.Send(Character.toString((Direction)));
        _currentSignal = Direction;
    }

}
