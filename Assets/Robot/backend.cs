using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Collections.Generic;

public class Coordinates
{
    public float px, py, pz, rx, ry, rz;

    public Coordinates(float px, float py, float pz, float rx, float ry, float rz)
    {
        this.px = px;
        this.py = py;
        this.pz = pz;
        this.rx = rx;
        this.ry = ry;
        this.rz = rz;
    }
}

public class AppData
{
    public string status;
    public int[] pathx;
    public int[] pathy;
    public int[] wallsx;
    public int[] wallsy;
}

public class BackendData
{
    public string status;
    public float x, y;
}

public class backend : MonoBehaviour
{
    // World Parameters
    public Coordinates origin;
    public Coordinates factor;
    public Coordinates robot;
    public Vector2 imageSize = new Vector2(300.0f, 220.0f);
    public Vector2 mapOrigin = new Vector2(10.0f, 10.0f);
    public float imageScale = 1.0f;

    // Velocity Params
    public Vector3 robotOrientation;
    public Vector3 robotLinearVelocity;
    public Vector3 robotAngularVelocity;
    public float linear_velocity = 1.0f;
    public float angular_velocity = 1.0f;
    public float gravity = 9.81f;
    public bool onFloor = false;
    

    // Communication Parameters 
    public List<int> pathxApp;
    public List<int> pathyApp;
    public List<float> pathxMap;
    public List<float> pathyMap;
    public string status;
    public float distThreshold;
    public float P, I, D;
    List<float> linearVel;
    List<float> angularVel;
    public GameObject goal;
    public GameObject next_point;
    public Rigidbody cart_rb;
    private float timeAcc = 0.0f;
    public float updateRate = 1.0f;

    // Communcation Information
    public int port = 8888; // Port to listen on
    public string TCPaddress = "127.0.0.1";
    private bool clientConnected = true;
    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;
    private byte[] buffer = new byte[1024*4];
    //System.Threading.Thread tcpListenerThread;

    private void Start()
    {

        // Initialization of variables (corresponding to Python constructor)
        origin = new Coordinates(41.5f, 0.0f, 11.5f, 0.0f, 0.0f, 0.0f);
        factor = new Coordinates(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
        robot = new Coordinates(41.5f, 0.0f, 11.5f, 0.0f, 0.0f, 0.0f);

        // Initialize lists and other variables as needed
        pathxApp = new List<int>();
        pathyApp = new List<int>();
        pathxMap = new List<float>();
        pathyMap = new List<float>();
        status = "Stop";
        distThreshold = 1.0f;
        P = 1.0f;
        I = 0.0f;
        D = 0.0f;
        linearVel = new List<float>();
        angularVel = new List<float>();

        // Define Rigidbody
        cart_rb = GetComponent<Rigidbody>();
        robotOrientation = new Vector3(-90,0,90);

        // Start listening for connections in a separate thread
        //tcpListenerThread = new System.Threading.Thread(ListenForTCP);
        //tcpListenerThread.Start();

        server = new TcpListener(IPAddress.Any, port);
        //server = new TcpListener(System.Net.IPAddress.Parse(TCPaddress), port);
        server.Start();
        Debug.Log("Socket listener started.");


        // Start listening for connections in a separate thread
        WaitForClient();
    }

    /*
        ROBOT FUNCTIONING
    */

    private void FixedUpdate()
    {
        // Update Goal Position
        if (pathxMap.Count > 0 && pathyMap.Count > 0)
        {
            goal.transform.position = new Vector3(pathxMap[pathxMap.Count - 1], 0.0f, pathyMap[pathyMap.Count - 1]);
            next_point.transform.position = new Vector3(pathxMap[0], 0.0f, pathyMap[0]);
        }

        // Send Robot Position
        if (timeAcc > updateRate)
        {
            BackendData data2update = new BackendData();
            data2update.status = status;
            data2update.y = (float)((-this.transform.position[0]/imageScale - 0.5 + imageSize[0]/2 ) - mapOrigin[0]);
            data2update.x = (float)((this.transform.position[2]/imageScale - 0.5 + imageSize[1]/2 ) - mapOrigin[1]);
            SendData(data2update);
            timeAcc = 0;
        }
        timeAcc += Time.deltaTime;

        // Move Robot

        
        if (status == "Continue")
        {
            
            // Compute distance and angle of next node
            if (pathxMap.Count > 0 && pathyMap.Count > 0)
            {
                // Direction
                float dirx = (float)(this.transform.position[0] - pathxMap[0]);
                float diry = (float)(this.transform.position[2] - pathyMap[0]);

                // Angular coordinates
                float distance = (float) Math.Sqrt(dirx*dirx + diry*diry);
                float angle = (float) (Math.Atan2(diry, dirx)*180/Math.PI);

                // Remove graph node if it close to certain threshold
                if (distance < distThreshold)
                {
                    pathxApp.RemoveAt(0);
                    pathyApp.RemoveAt(0);
                    pathxMap.RemoveAt(0);
                    pathyMap.RemoveAt(0);
                }

            
                float yaw = (float)(-angle + 90 - (robotOrientation[2]));
                Debug.Log("Direction = (" + dirx + "," +diry + ") | Distance = " + distance + " | Angle = " + angle + " Rotation Z = " + (robotOrientation[2]) + "| Yaw = " + yaw);

                /*
                if (yaw > Math.PI) // Convert the range to -pi to pi
                {
                    yaw = yaw - 2*Math.PI;
                }
                else if (yaw  < -Math.PI)
                {
                    yaw = yaw + 2*Math.PI;
                }*/

                
                // Pid
                if (Math.Sqrt(yaw*yaw) < 180/16)
                {
                    linear_velocity = P*(float)Math.Min(Math.Max(distance, 0.2), 1.4);
                }
                else
                {
                    linear_velocity = 0.0f;
                }
                
                if (Math.Sqrt(yaw*yaw) > 180/16)
                {
                    if (yaw > 0.0)
                    {
                        angular_velocity = 1.2f;
                    }
                    else
                    {
                        angular_velocity = -1.2f;
                    }

                    robotOrientation = robotOrientation + new Vector3(0, 0, 1)*angular_velocity;
                }
                else
                {
                    angular_velocity = 0.0f;
                    robotOrientation = new Vector3(-90, 0, -angle + 90);
                }

                
            }
            else
            {
                status = "Destination";
            }
        }
        else
        {
            linear_velocity = 0.0f;
            angular_velocity = 0.0f;
        }

        // Rotation and Velocity
        if (onFloor)
        {
            cart_rb.velocity = this.transform.up * linear_velocity;
            cart_rb.rotation = Quaternion.Euler(robotOrientation);
        }
        else
        {
            cart_rb.velocity = - this.transform.forward * gravity;
        }
        
        
    }


    /*
        COMMUNICATION BETWEEN THE APP AND THE ROBOT
    */

    private async void WaitForClient()
    {
        try
        {
            client = await server.AcceptTcpClientAsync();
            stream = client.GetStream();
            Debug.Log("Client connected.");

            // Start receiving data from the Python application
            ReceiveData();
        }
        catch (Exception ex)
        {
            Debug.LogError("Error connecting to client: " + ex.Message);
        }
    }

    private async void ReceiveData()
    {
        try
        {
            while (clientConnected)
            {
                int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string jsonString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    Debug.Log("Received data: " + jsonString);

                    // Deserialize the JSON string to retrieve the structure
                    try
                    {
                        AppData data = JsonUtility.FromJson<AppData>(jsonString);
                        
                        // Process the received data
                        status = data.status;

                        if (data.status == "Next Item" || data.status == "Item Path" || data.status == "Destination")
                        {
                            // Clear lists
                            pathxApp.Clear();
                            pathyApp.Clear();
                            pathxMap.Clear();
                            pathyMap.Clear();

                            // Update paths
                            for (int i=0; i < Math.Min(data.pathx.Length, data.pathy.Length); i++)
                            {
                                pathxApp.Add((int) data.pathy[i]);
                                pathyApp.Add((int) data.pathx[i]);
                                pathxMap.Add((float)(-((data.pathy[i] + mapOrigin[0]) - imageSize[0]/2 + 0.5)*imageScale));
                                pathyMap.Add((float)(-((-data.pathx[i] - mapOrigin[1]) + imageSize[1]/2 - 0.5)*imageScale));
                            }

                            // Update robot status
                            if (data.status == "Item Path"){status = "Stop";}
                            else {status = "Continue";}
                        }

                        // Update robot status
                        if (data.status == "Pause"){status = "Pause";}
                        else if (data.status == "Resume"){status = "Continue";}
                    }
                    catch (Exception e)
                    {
                        Debug.LogError("Error deserializing JSON: " + e.Message);
                    }
                }
                else
                {
                    Debug.Log("Client disconnected.");
                    HandleDisconnect();
                    break; 
                }

                Array.Clear(buffer, 0, buffer.Length); // Clear the buffer
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("Error in ReceiveData(): " + ex.Message);
            HandleDisconnect();
        }
    }

    // Function to send data to the Application
    public void SendData(BackendData data)
    {
        if (stream != null)
        {
            string serializedData = JsonUtility.ToJson(data);
            byte[] byteData = Encoding.UTF8.GetBytes(serializedData);
            stream.Write(byteData, 0, byteData.Length);
        }
    }

    // Handle Disconnects
    private void HandleDisconnect()
    {
        // Perform cleanup or other actions when client disconnects
        clientConnected = false;
        // Close the network stream, client, and perform any other necessary cleanup
        if (stream != null)
            stream.Close();

        if (client != null)
            client.Close();
    }

    private void OnDestroy()
    {
        stream.Close();
        client.Close();
        server.Stop();
    }

    private void OnCollisionStay(Collision collision)
    {
        
        if (collision.collider.CompareTag("Floor"))
        {
            onFloor = true;
        }
        else
        {
            onFloor = false;
        }
    }
}
