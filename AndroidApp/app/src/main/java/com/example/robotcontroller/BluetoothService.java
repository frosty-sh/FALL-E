package com.example.robotcontroller;

import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Handler;
import android.os.Looper;
import android.widget.Toast;

import java.io.IOException;
import java.io.OutputStream;
import java.util.UUID;


public class BluetoothService {

    private final Activity _activity;

    private static final UUID DEVICE_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); // "random" unique identifier
    private BluetoothAdapter _BTAdapter; //Bluetooth adapter
    private ConnectedThread _connectedThread; // bluetooth background worker thread to send and receive data
    private BluetoothSocket _BTSocket = null; // bi-directional client-to-client data path

    BluetoothService(Activity activity) {
        _activity = activity;
        _BTAdapter = BluetoothAdapter.getDefaultAdapter(); // get a handle on the bluetooth radio


        if (!_BTAdapter.isEnabled()) {
            _BTAdapter.enable();

            //Restart activity
            Intent i = _activity.getApplicationContext().getPackageManager().
                    getLaunchIntentForPackage(activity.getApplicationContext().getPackageName());
            i.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
            i.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
            }
            _activity.startActivity(i);

            return;
        }

        final String address = "98:D3:32:30:AD:C2";
        final String name = "RobotBT";

        // Spawn a new thread to avoid blocking the GUI
        new Thread() {
            public void run() {
                boolean fail = false;

                //Get the device
                BluetoothDevice device = _BTAdapter.getRemoteDevice(address);

                //Create socket with device
                try {
                    _BTSocket = device.createInsecureRfcommSocketToServiceRecord(DEVICE_UUID);
                } catch (IOException e) {
                    fail = true;
                }

                // Establish the Bluetooth socket connection.
                try {
                    _BTSocket.connect();
                } catch (IOException e) {
                    try {
                        fail = true;
                        _BTSocket.close();
                    } catch (IOException e2) { }
                }

                if (!fail) {
                    //Start new connection thread
                    _connectedThread = new ConnectedThread(_BTSocket);
                    _connectedThread.start();
                    //Send toast to UI thread
                    new Handler(Looper.getMainLooper()).post(new Runnable() {
                        @Override
                        public void run() {
                            Toast.makeText(_activity.getApplicationContext(), "Connected to Robot!", Toast.LENGTH_SHORT).show();
                        }
                    });
                } else {
                    //Send alert to UI thread and colse the app
                    new Handler(Looper.getMainLooper()).post(new Runnable() {
                        @Override
                        public void run() {
                            AlertDialog.Builder builder1 = new AlertDialog.Builder(_activity);
                            builder1.setTitle("Device not found!");
                            builder1.setMessage("Make sure that the bluetooth on the robot is turned on and restart the app");
                            builder1.setCancelable(true);
                            builder1.setNeutralButton(android.R.string.ok,
                                    new DialogInterface.OnClickListener() {
                                        public void onClick(DialogInterface dialog, int id) {
                                            dialog.cancel();
                                            _activity.finish();
                                            System.exit(0);
                                        }
                                    });

                            AlertDialog alert11 = builder1.create();
                            alert11.show();

                        }
                    });
                }
            }
        }.start();

    }

    //Wrapper for sending data to paired device
    public void Send(String input) {
        _connectedThread.write(input);
    }

    private class ConnectedThread extends Thread {
        private final BluetoothSocket __socket;
        private final OutputStream __outStream;

        public ConnectedThread(BluetoothSocket socket) {
            __socket = socket;
            OutputStream tmpOut = null;

            // Output stream, using temp objects because
            // member stream is final
            try {
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
            }

            __outStream = tmpOut;
        }

        public void write(String input) {
            byte[] bytes = input.getBytes();//converts entered String into bytes
            try {
                __outStream.write(bytes);
            } catch (IOException e) {
            }
        }

        /* Closes the connection */
        public void cancel() {
            try {
                __socket.close();
            } catch (IOException e) {
            }
        }
    }

}

