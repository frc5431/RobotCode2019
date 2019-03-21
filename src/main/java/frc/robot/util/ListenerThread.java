package frc.robot.util;

import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.function.Consumer;

public class ListenerThread extends Thread{
    private final int port;
    private final Consumer<String> handler;

    public ListenerThread(final int port, final Consumer<String> handler){
        this.port = port;
        this.handler = handler;
    }

    @Override
    public void run(){
        Titan.l("Launching listener thread on port " + port);
        try(final ServerSocket server = new ServerSocket(port)){
            Socket socket = null;
            while(true){
                Titan.l("Waiting for connection to port " + port);
                socket = server.accept();
                Titan.l("Connected on port " + port);
                try(final PrintWriter writer = new PrintWriter(socket.getOutputStream())){
                    try(final BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()))){
                        String message;
                        while(socket.isConnected()){
                            message = reader.readLine();
                            if(message == null){
                                break;
                            }
                            handler.accept(message);
                        }
                    }
                }
            }
        }catch(final IOException ex){
            ex.printStackTrace();
        }
    }

}