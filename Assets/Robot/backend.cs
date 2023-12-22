using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class App2Backend
{
    public string message { get; set; }
    public int[] numbers { get; set; }
}

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

    Coordinates origin;
    Coordinates factor;
    Coordinates robot;
    Vector2 imageSize;
    Vector2 mapOrigin;
    float imageScale;
    TcpListener server;

    List<int> pathxApp;
    List<int> pathyApp;
    List<int> pathxMap;
    List<int> pathyMap;
    string status;
    float distThreshold;
    float P, I, D;
    List<float> linearVel;
    List<float> angularVel;

    // Update is called once per frame
    void Update()
    {
        
    }

    public int port = 8888; // Port to listen on
    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;
    private byte[] buffer = new byte[1024];

    private void Start()
    {

        // Initialization of variables (corresponding to Python constructor)
        origin = new Coordinates(-41.5f, -11.5f, 1.0f, 0.0f, 0.0f, 0.0f);
        factor = new Coordinates(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
        robot = new Coordinates(-41.5f, -11.5f, 1.0f, 0.0f, 0.0f, 0.0f);
        imageSize = new Vector2(300.0f, 220.0f);
        mapOrigin = new Vector2(10.0f, 10.0f);
        imageScale = 1.0f;

        // TCP Server setup
        server = new TcpListener(System.Net.IPAddress.Any, 8888);
        server.Start();

        // Initialize lists and other variables as needed
        pathxApp = new List<int>();
        pathyApp = new List<int>();
        pathxMap = new List<int>();
        pathyMap = new List<int>();
        status = "Stop";
        distThreshold = 1.0f;
        P = 1.0f;
        I = 0.0f;
        D = 0.0f;
        linearVel = new List<float>();
        angularVel = new List<float>();

        // Start listening for connections in a separate thread
        System.Threading.Thread tcpListenerThread = new System.Threading.Thread(ListenForTCP);
        tcpListenerThread.Start();

        
        server = new TcpListener(IPAddress.Any, port);
        server.Start();
        Debug.Log("Socket listener started.");

        // Start listening for connections in a separate thread
        WaitForClient();
    }

    private async void WaitForClient()
    {
        client = await server.AcceptTcpClientAsync();
        stream = client.GetStream();
        Debug.Log("Client connected.");

        // Start receiving data from the Python application
        ReceiveData();
    }

    private async void ReceiveData()
    {
        while (true)
        {
            int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
            string jsonString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
            Debug.Log("Received data: " + jsonString);

            // Deserialize the JSON string to retrieve the structure
            try
            {
                App2Backend receivedData = JsonUtility.FromJson<App2Backend>(jsonString);
                // Process the received data as needed
                Debug.Log(receivedData);
            }
            catch (Exception e)
            {
                Debug.LogError("Error deserializing JSON: " + e.Message);
            }
        }
    }

    // Function to send data to Python
    public void SendDataToPython(App2Backend data)
    {
        if (stream != null)
        {
            string serializedData = JsonUtility.ToJson(data);
            byte[] byteData = Encoding.UTF8.GetBytes(serializedData);
            stream.Write(byteData, 0, byteData.Length);
        }
    }

    private void OnDestroy()
    {
        stream.Close();
        client.Close();
        server.Stop();
    }
}
