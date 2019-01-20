package frc.robot.vision;

import java.io.IOException;
import java.net.SocketException;

public class EchoServer extends Thread {
 
    private java.net.DatagramSocket socket;
    private boolean running;
    private byte[] buf = new byte[256];
 
    public EchoServer() throws SocketException{
        socket = new java.net.DatagramSocket(5005);
    }
 
    public void run() {
        running = true;
 
        while (running) {
            try{
                java.net.DatagramPacket packet = new java.net.DatagramPacket(buf, buf.length);
                socket.receive(packet);
                
                java.net.InetAddress address = packet.getAddress();
                int port = packet.getPort();
                packet = new java.net.DatagramPacket(buf, buf.length, address, port);
              //  String received = new String(packet.getData(), 0, packet.getLength());
                
                /*JSONArray jsonArray = new JSONArray(received);
                socket.send(packet);
                socket.close();
                */
            }catch(final IOException e){
                e.printStackTrace();
            }
        }
        
    }
}

