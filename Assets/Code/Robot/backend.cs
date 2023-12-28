using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Collections.Generic;
using System.IO;

/*
    CLASS PARAMETERS
*/

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
    public string mode;
    public string status;
    public int[] pathx;
    public int[] pathy;
    public int[] productsx;
    public int[] productsy;
    public string[] productNames;
    public string mapName;
}

public class BackendData
{
    public string status;
    public float x, y;
    public float xUser, yUser;
}

/*
    BACKEND
*/

public class backend : MonoBehaviour
{
    // World Parameters
    public Coordinates origin;
    public Coordinates factor;
    public Coordinates robot;
    public Vector2 imageSize = new Vector2(300.0f, 220.0f);
    public Vector2 mapOrigin = new Vector2(10.0f, 10.0f);
    public float imageScale = 1.0f;
    public GameObject user;
    public User userBehaviour;
    public GameObject[] maps;
    public GameObject productPrefab;
    private GameObject[] shelves3;
    private GameObject[] shelves4;
    private string shopName="Model3";

    // Velocity Params
    public Vector3 robotOrientation;
    public Vector3 robotLinearVelocity;
    public Vector3 robotAngularVelocity;
    public float linear_velocity = 1.0f;
    public float angular_velocity = 1.0f;
    public float linearVelMax = 1.4f;
    public float gravity = 9.81f;
    public bool onFloor = false;

    // Lidar
    [SerializeField]
    private LayerMask layerMask;
    [SerializeField]
    [Min(1)]
    public float range = 2.0f;
    public float UserRange = 10.0f;
    private RaycastHit hit1;
    private RaycastHit hit2;
    private RaycastHit hit3;
    private RaycastHit hit4;
    private RaycastHit hit5;
    private RaycastHit hit1b;
    private RaycastHit hit2b;
    private RaycastHit hit3b;
    private RaycastHit hit4b;
    private RaycastHit hit5b;
    private RaycastHit hitUser;

    

    // Communication Parameters 
    public List<int> pathxApp;
    public List<int> pathyApp;
    public List<float> pathxMap;
    public List<float> pathyMap;
    public string status;
    public float distThreshold;
    public float distThresholdLast;
    public float P, I, D;
    public GameObject goal;
    public GameObject next_point;
    public Rigidbody cart_rb;
    private float timeAcc = 0.0f;
    public float updateRate = 1.0f;

    // Communication Information
    public int port = 8888; // Port to listen on
    public string TCPaddress = "127.0.0.1";
    private bool clientConnected = true;
    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;
    private byte[] buffer = new byte[1024*4];
    //System.Threading.Thread tcpListenerThread;

    // Log Files
    private string logFilePath;
    private string logFilePathApp;

    // Paths
    public GameObject pathPrefab;
    public GameObject[] pathPoints;
    public bool updatePath = true;

    private void Start()
    {
        /*
            Initialize the robot configuration and communication when the Unity Environment starts
        */

        // Initialization of variables (corresponding to Python constructor)
        // origin = new Coordinates(41.5f, 0.0f, 11.5f, 0.0f, 0.0f, 0.0f);
        // factor = new Coordinates(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
        // robot = new Coordinates(41.5f, 0.0f, 11.5f, 0.0f, 0.0f, 0.0f);

        // Initialize lists and other variables as needed
        pathxApp = new List<int>();
        pathyApp = new List<int>();
        pathxMap = new List<float>();
        pathyMap = new List<float>();
        status = "Stop";
        distThreshold = 0.1f;
        distThresholdLast = 1.4f;
        P = 1.0f;
        I = 0.0f;
        D = 0.0f;
        userBehaviour = user.GetComponent<User>();

        // Define Rigidbody
        cart_rb = GetComponent<Rigidbody>();
        maps = GameObject.FindGameObjectsWithTag("Shop");
        shelves3 = GameObject.FindGameObjectsWithTag("shelve_shop3");
        shelves4 = GameObject.FindGameObjectsWithTag("shelve_shop4");
        foreach(GameObject goMap in maps)
        {
            if (goMap.name == "shop3")
            {
                goMap.SetActive(true);
            }
            else
            {
                goMap.SetActive(false);
            }
        }

        robotOrientation = new Vector3(-90,0,90);

        // Initialize Log File
        DateTime currentTime = DateTime.UtcNow;
        Debug.Log(String.Format("Log_{0:dMyyyy_HHmmss}", currentTime));
        logFilePath = Application.dataPath + "/LogFiles/" + String.Format("Log_{0:dMyyyy_HHmmss}.txt", currentTime);
        logFilePathApp = Application.dataPath + "/LogFiles/" + String.Format("LogApp_{0:dMyyyy_HHmmss}.txt", currentTime);
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Environment Initialization");
        LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Environment Initialization (App Communication)");


        // Initialize App Communication
        server = new TcpListener(IPAddress.Any, port);
        //server = new TcpListener(System.Net.IPAddress.Parse(TCPaddress), port);
        server.Start();
        Debug.Log("Socket listener started.");
        
        currentTime = DateTime.UtcNow;
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Socket listener started");
        LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Socket listener started");
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Socket Conf |ip:" + TCPaddress + "|port:" + port );
        LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Socket Conf |ip:" + TCPaddress + "|port:" + port);

        // Update data
        currentTime = DateTime.UtcNow;
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Configuration |imageSize:" + imageSize +
                                "|mapOrigin:" + mapOrigin + "|imageScale:" + imageScale + "|linearVelMax:" + linearVelMax +
                                "|gravity:" + gravity + "|range:" + range + "|distThr:" + distThreshold + "|distThrLast:" + distThresholdLast +
                                "|P:" + P + "|I" + I + "|D" + D + "|updateRate:" + updateRate);


        // Start listening for connections in a separate thread
        WaitForClient();
    }

    /*
        ROBOT FUNCTIONING
    */

    private void FixedUpdate()
    {
        /*
            Control the robot during each cycle of Unity
        */

        // Update Goal Position
        if (pathxMap.Count > 0 && pathyMap.Count > 0)
        {
            goal.transform.position = new Vector3(pathxMap[pathxMap.Count - 1], 0.0f, pathyMap[pathyMap.Count - 1]);
            next_point.transform.position = new Vector3(pathxMap[0], 0.0f, pathyMap[0]);

            if (status == "FollowMe" || status == "FollowMeWait" || status == "FollowMeBack")
            {
                goal.SetActive(false);
                next_point.SetActive(false);
            }
            else
            {
                goal.SetActive(true);
                next_point.SetActive(true);
            }
        }

        // Lidar Draws        
        Debug.DrawRay(transform.position, (-transform.right + transform.up) * range / (float)Math.Sqrt(2), Color.black);
        Debug.DrawRay(transform.position, (-(float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up) * range, Color.red);
        Debug.DrawRay(transform.position, transform.up * range, Color.red);
        Debug.DrawRay(transform.position, ((float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up) * range, Color.red);
        Debug.DrawRay(transform.position, (transform.right + transform.up) * range / (float)Math.Sqrt(2), Color.yellow);
        Debug.DrawRay(transform.position, -(-transform.right + transform.up) * range / (float)Math.Sqrt(2), Color.black);
        Debug.DrawRay(transform.position, -(-(float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up) * range, Color.blue);
        Debug.DrawRay(transform.position, -transform.up * range, Color.blue);
        Debug.DrawRay(transform.position, -((float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up) * range, Color.blue);
        Debug.DrawRay(transform.position, -(transform.right + transform.up) * range / (float)Math.Sqrt(2), Color.yellow);

        // Lidar detections
        bool detect1 = Physics.Raycast(transform.position, (-transform.right + transform.up) / (float)Math.Sqrt(2), out hit1, range, layerMask);
        bool detect2 = Physics.Raycast(transform.position, (-(float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up), out hit2, range, layerMask);
        bool detect3 = Physics.Raycast(transform.position, transform.up, out hit3, range, layerMask);
        bool detect4 = Physics.Raycast(transform.position, ((float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up), out hit4, range, layerMask);
        bool detect5 = Physics.Raycast(transform.position, (transform.right + transform.up) / (float)Math.Sqrt(2), out hit5, range, layerMask);
        bool detect1b = Physics.Raycast(transform.position, -(-transform.right + transform.up) / (float)Math.Sqrt(2), out hit1b, range, layerMask);
        bool detect2b = Physics.Raycast(transform.position, -(-(float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up), out hit2b, range, layerMask);
        bool detect3b = Physics.Raycast(transform.position, -transform.up, out hit3b, range, layerMask);
        bool detect4b = Physics.Raycast(transform.position, -((float)Math.Sin(Math.PI/8)*transform.right + (float)Math.Cos(Math.PI/8)*transform.up), out hit4b, range, layerMask);
        bool detect5b = Physics.Raycast(transform.position, -(transform.right + transform.up) / (float)Math.Sqrt(2), out hit5b, range, layerMask);


        bool userInPath = false;
        bool costumerInPath = false;
        bool sthBack = false;
        if (status == "UserInPath" || status == "Wait")
        {
            status = "Continue";
        }
        else if (status == "FollowMeWait")
        {
            status = "FollowMe";
        }

        // Lidar detects
        if(detect1)
        { if (hit1.collider.tag == "User")
            { userInPath = true;}
            else if (hit1.collider.tag == "Costumer")
            { costumerInPath = true;}}
        if(detect2)
        { if (hit2.collider.tag == "User")
            { userInPath = true;}
          else if (hit2.collider.tag == "Costumer")
            { costumerInPath = true;}}
        if(detect3)
        { if (hit3.collider.tag == "User")
            { userInPath = true;}
          else if (hit3.collider.tag == "Costumer")
            { costumerInPath = true;}}
        if(detect4)
        { if (hit4.collider.tag == "User")
            { userInPath = true;}
          else if (hit4.collider.tag == "Costumer")
            { costumerInPath = true;}}
        if(detect5)
        { if (hit5.collider.tag == "User")
            { userInPath = true;}
          else if (hit5.collider.tag == "Costumer")
            { costumerInPath = true;}}
        if(detect2b || detect3b || detect4b)
        { sthBack = true;}

        // Lidar Update status
        if (userInPath && status == "Continue")
        {
            status = "UserInPath";
        }
        else if (userInPath && status == "FollowMe")
        {
            Vector3 distFollow = this.transform.position - user.transform.position;
            float distFollowValue = (float) Math.Sqrt(Math.Pow(distFollow[0], 2) + Math.Pow(distFollow[2], 2));

            if((distFollowValue < range - 0.5) && (sthBack == false))
            {
                status = "FollowMeBack";
            }
            else 
            {
                status = "FollowMeWait";
            }      
        }

        // Camera/Lidar detection of user
        Debug.DrawRay(transform.position, (user.transform.position - this.transform.position) * UserRange / Vector3.Distance(user.transform.position, this.transform.position), Color.green);
        bool detectUserProximity = Physics.Raycast(transform.position, (user.transform.position - this.transform.position)/Vector3.Distance(user.transform.position, this.transform.position), out hitUser, UserRange, layerMask);
        if(status=="Continue")
        {   if (detectUserProximity)
            {   if (hitUser.collider.tag == "User"){ status="Continue";}
                else {status="Wait";}}
            else {status="Wait";}}

        // Send Robot Position
        if (timeAcc > updateRate)
        {
            BackendData data2update = new BackendData();
            data2update.status = status;
            data2update.y = (float)((-this.transform.position[0]/imageScale - 0.5 + imageSize[0]/2 ) - mapOrigin[0]);
            data2update.x = (float)((this.transform.position[2]/imageScale - 0.5 + imageSize[1]/2 ) - mapOrigin[1]);
            data2update.yUser = (float)((-user.transform.position[0]/imageScale - 0.5 + imageSize[0]/2 ) - mapOrigin[0]);
            data2update.xUser = (float)((user.transform.position[2]/imageScale - 0.5 + imageSize[1]/2 ) - mapOrigin[1]);
            SendData(data2update);
            timeAcc = 0;
        }

        // Move Robot

        // Compute distance and angle of next node
        if (pathxMap.Count > 0 && pathyMap.Count > 0)
        {
            // Direction
            float dirx = (float)(this.transform.position[0] - pathxMap[0]);
            float diry = (float)(this.transform.position[2] - pathyMap[0]);

            // Angular coordinates
            float distance = (float) Math.Sqrt(dirx*dirx + diry*diry);
            float angle = (float) (Math.Atan2(diry, dirx)*180/Math.PI);

            if (status != "Stop" && status != "Wait")
            {

                // Remove graph node if it close to certain threshold
                if (pathxMap.Count == 1 && pathyMap.Count == 1 && distance < distThresholdLast)
                {
                    pathxApp.RemoveAt(0);
                    pathyApp.RemoveAt(0);
                    pathxMap.RemoveAt(0);
                    pathyMap.RemoveAt(0);
                }
                if (distance < distThreshold)
                {
                    pathxApp.RemoveAt(0);
                    pathyApp.RemoveAt(0);
                    pathxMap.RemoveAt(0);
                    pathyMap.RemoveAt(0);
                }

            
                float yaw = (float)(-angle + 90 - (robotOrientation[2]));
                if (yaw > 180)
                {
                    yaw = yaw - 360;
                }
                else if (yaw < -180)
                {
                    yaw = 360 + yaw;
                }

                //Linear Movement
                if ((status == "Continue" || status == "FollowMe") && (costumerInPath == false))
                {
                    // Pid
                    if (Math.Sqrt(yaw*yaw) < 180/16)
                    {
                        if (pathxMap.Count > 1 && pathyMap.Count > 1)
                        {
                            linear_velocity = P*linearVelMax;
                        }
                        else
                        {
                            linear_velocity = P*(float)Math.Min(Math.Max(distance, 0.2), linearVelMax);
                        }
                    }
                    else
                    {
                        linear_velocity = 0.0f;
                    }
                }
                else if (status == "FollowMeBack")
                {
                    linear_velocity = -P*linearVelMax;
                
                }
                else
                {
                    linear_velocity = 0.0f;
                }

                // Angular Movement
                if (status == "Continue" || status == "FollowMe" || status == "UserInPath")
                {
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
                    angular_velocity = 0.0f;
                }
            }
            else
            {
                linear_velocity = 0.0f;
                angular_velocity = 0.0f;
            }

            
        }
        else if (status == "Continue")
        {
            status = "Destination";
        }
        else if (status == "Destination")
        {
            bool sthFrontAside = false;
            bool sthBackAside = false;
            bool userFrontAside = false;
            bool userBackAside = false;
            bool userFrontAsideL = false;
            bool userBackAsideL = false;
            bool userFrontAsideR = false;
            bool userBackAsideR = false;

            if (detect2 || detect3 || detect4)
            { sthFrontAside = true;}
            if (detect2b || detect3b || detect4b)
            { sthBackAside = true;}

            if(detect1){if (hit1.collider.tag == "User"){userFrontAside = true; userFrontAsideL = true;}}
            if(detect2){if (hit2.collider.tag == "User"){userFrontAside = true; userFrontAsideL = true;}}
            if(detect3){if (hit3.collider.tag == "User"){userFrontAside = true;}}
            if(detect4){if (hit4.collider.tag == "User"){userFrontAside = true; userFrontAsideR = true;}}
            if(detect5){if (hit5.collider.tag == "User"){userFrontAside = true; userFrontAsideR = true;}}

            if(detect1b){if (hit1b.collider.tag == "User"){userBackAside = true; userBackAsideL = true;}}
            if(detect2b){if (hit2b.collider.tag == "User"){userBackAside = true; userBackAsideL = true;}}
            if(detect3b){if (hit3b.collider.tag == "User"){userBackAside = true;}}
            if(detect4b){if (hit4b.collider.tag == "User"){userBackAside = true; userBackAsideR = true;}}
            if(detect5b){if (hit5b.collider.tag == "User"){userBackAside = true; userBackAsideR = true;}}


            if (userFrontAside && (sthBackAside == false))
            {
                linear_velocity = -2.0f;
                angular_velocity = 0.0f;
            }
            else if (userBackAside && (sthFrontAside == false))
            {
                linear_velocity = 2.0f;
                angular_velocity = 0.0f;
            }
            else if (userFrontAside && sthBackAside)
            {
                linear_velocity = 0.0f;
                if (userFrontAsideL) { angular_velocity = 1.2f;}
                else{ angular_velocity = -1.2f;}
                robotOrientation = robotOrientation + new Vector3(0, 0, 1)*angular_velocity;
                
            }
            else if (userBackAside && sthFrontAside)
            {
                linear_velocity = 0.0f;
                if (userBackAsideL) { angular_velocity = 1.2f;}
                else{ angular_velocity = -1.2f;}
                robotOrientation = robotOrientation + new Vector3(0, 0, 1)*angular_velocity;
            }
            else
            {
                linear_velocity = 0.0f;
                angular_velocity = 0.0f;
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

        // Update data
        DateTime currentTime = DateTime.UtcNow;
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Robot data |status:" + status +
                                "|robotOrientationX:" + robotOrientation[0] + "|robotOrientationY:" + robotOrientation[1] + "|robotOrientationZ:" + robotOrientation[2] +
                                "|robotLocX:" + this.transform.position[0] + "|robotLocY:" + this.transform.position[1] + "|robotLocZ:" + this.transform.position[2] +
                                "|grounded:" + onFloor + "|linearVel:" + linear_velocity + "|angularVel:" + angular_velocity);
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Path data " +
                                "|pathXapp:" + string.Join("&",pathxApp) + "|pathYapp:" + string.Join("&",pathyApp) + "|pathXmap:" + string.Join("&",pathxMap) + "|pathYmap:" + string.Join("&",pathyMap));
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Lidar data|userInPath:" + userInPath + "|sthBack:" + sthBack +
                                "|detect1:" + detect1 + "|detect2:" + detect2 + "|detect3:" + detect3 + "|detect4:" + detect4 + "|detect5:" + detect5 +
                                "|detect1b:" + detect1b + "|detect2b:" + detect2b + "|detect3b:" + detect3b + "|detect4b:" + detect4b + "|detect5b:" + detect5b);
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - User data |userSpeed:" + userBehaviour.userSpeed +
                                "|userRotX:" + user.transform.rotation[0] + "|userRotY:" + user.transform.rotation[1] + "|userRotZ:" + user.transform.rotation[2] + "|userRotW:" + user.transform.rotation[3] +
                                "|userLocX:" + user.transform.position[0] + "|userLocY:" + user.transform.position[1] + "|userLocZ:" + user.transform.position[2]);
        LogToFile(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Product data " +
                                "|Count:" + userBehaviour.productCount + "|CollCount:" + userBehaviour.productCollCount +
                                "|List:" + string.Join("&",userBehaviour.productList) + "|Collected:" + string.Join("&",userBehaviour.productCollected));

        
        timeAcc += Time.deltaTime;

        // Update path
        if (status == "Continue" || status == "Wait")
        {
            Debug.Log("Updatable path");
            if (updatePath)
            {
                pathPoints = GameObject.FindGameObjectsWithTag("PathPoints");
                foreach(GameObject p in pathPoints) { Destroy(p); }

                for(int k=0; k < pathxMap.Count; k++)
                {
                    Instantiate(pathPrefab, new Vector3(pathxMap[k], 5 ,pathyMap[k]), Quaternion.identity);
                }
                updatePath = false;
                Debug.Log("Updated path");
            }
        }
        else
        {
            updatePath = true;
        }
        
        
    }


    /*
        COMMUNICATION BETWEEN THE APP AND THE ROBOT
    */

    private async void WaitForClient()
    {
        /*
            Waits for a client in a different thread to start the robot's communication with the application
        */

        try
        {
            client = await server.AcceptTcpClientAsync();
            stream = client.GetStream();
            Debug.Log("Client connected.");

            DateTime currentTime = DateTime.UtcNow;
            LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Client Connected");

            // Start receiving data from the Python application
            ReceiveData();
        }
        catch (Exception ex)
        {
            Debug.LogError("Error connecting to client: " + ex.Message);

            DateTime currentTime = DateTime.UtcNow;
            LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + " - Error Connecting the Client");
        }
    }

    private async void ReceiveData()
    {
        /*
            Receives data from the application and updates the internal state of the robot
        */

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

                        //Update data

                        DateTime currentTime = DateTime.UtcNow;
                        LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + 
                                    " - App Data |mode:" + data.mode + "|status:" +data.status + 
                                    "|pathx:" + string.Join("&",data.pathx) + "|pathy:" + string.Join("&",data.pathy) + 
                                    "|productsx:" + string.Join("&",data.productsx) + "|productsy:" + string.Join("&",data.productsy) + "|products:" + string.Join("&",data.productNames) + 
                                    "|map:" + data.mapName) ;

                        if (data.status == "World Init")
                        {
                            shopName = data.mapName;

                            // Enable maps
                            foreach(GameObject goMap in maps)
                            {
                                if (data.mapName == "Model3" && goMap.name == "shop3")
                                {
                                    goMap.SetActive(true);
                                }
                                else if (data.mapName == "Model4" && goMap.name == "shop4")
                                {
                                    goMap.SetActive(true);
                                }
                                else
                                {
                                    goMap.SetActive(false);
                                }
                            }

                            // Place Robot and user
                            float posX = (float)(-((data.pathy[0] + mapOrigin[0]) - imageSize[0]/2 + 0.5)*imageScale);
                            float posY = (float)(-((-data.pathx[0] - mapOrigin[1]) + imageSize[1]/2 - 0.5)*imageScale);

                            if (data.mapName == "Model3")
                            {
                                this.transform.position = new Vector3(posX, 1.0f, posY);
                                user.transform.position = new Vector3(posX, 1.0f, posY - 2);
                            }
                            else if (data.mapName == "Model4")
                            {
                                this.transform.position = new Vector3(posX, 1.0f, posY);
                                user.transform.position = new Vector3(posX, 1.0f, posY + 2);
                            }


                            // Update robot status
                            status = "Stop";
                        }
                        else if (data.status == "FollowMe")
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

                            // Remove first position
                            pathxApp.RemoveAt(0);
                            pathyApp.RemoveAt(0);
                            pathxMap.RemoveAt(0);
                            pathyMap.RemoveAt(0);

                            // Update Status
                            status = "FollowMe";
                        }
                        else if (data.status == "Next Item" || data.status == "Item Path" || data.status == "Destination")
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

                            // Remove first position
                            pathxApp.RemoveAt(0);
                            pathyApp.RemoveAt(0);
                            pathxMap.RemoveAt(0);
                            pathyMap.RemoveAt(0);

                            // Update robot status
                            if (data.status == "Item Path"){status = "Stop";}
                            else {status = "Continue";}

                            if (data.status == "Item Path")
                            {
                                // Place the products if product list
                                GameObject[] products = GameObject.FindGameObjectsWithTag("Product");
                                foreach(GameObject goprod in products)
                                {
                                    Destroy(goprod);
                                }
                                Debug.Log("Previous products removed");

                                currentTime = DateTime.UtcNow;
                                LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + 
                                    " - Previous products removed") ;

                                for(int p=0; p < Math.Min(Math.Min(data.productsx.Length, data.productsy.Length), data.productNames.Length); p++)
                                {
                                    productPrefab.name = data.productNames[p];
                                    productPrefab.tag = "Product";

                                    float posXprod = (float)(-((data.productsy[p] + mapOrigin[0]) - imageSize[0]/2 + 0.5)*imageScale);
                                    float posYprod = (float)(-((-data.productsx[p] - mapOrigin[1]) + imageSize[1]/2 - 0.5)*imageScale);
                                    float posXshelf = posXprod;
                                    float posYshelf = posYprod;

                                    
                                    if (shopName == "Model3")
                                    {
                                        float dist = 3.0f;
                                        foreach (GameObject shelf in shelves3)
                                        {
                                            float shelf2prod = (float)Math.Sqrt(Math.Pow(posXprod - shelf.transform.position[0], 2) + Math.Pow(posYprod - shelf.transform.position[2], 2));
                                            if (dist > shelf2prod)
                                            {
                                                dist = shelf2prod;
                                                posXshelf = shelf.transform.position[0];
                                                posYshelf = shelf.transform.position[2];

                                                if (shelf2prod < 1.0f)
                                                {
                                                    posXshelf = posXprod;
                                                    posYshelf = posYprod;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else if (shopName == "Model4")
                                    {
                                        float dist = 10.0f;
                                        foreach (GameObject shelf in shelves4)
                                        {
                                            float shelf2prod = (float)Math.Sqrt(Math.Pow(posXprod - shelf.transform.position[0], 2) + Math.Pow(posYprod - shelf.transform.position[2], 2));
                                            if (dist > shelf2prod)
                                            {
                                                dist = shelf2prod;
                                                posXshelf = shelf.transform.position[0];
                                                posYshelf = shelf.transform.position[2];

                                                if (shelf2prod < 1.0f)
                                                {
                                                    posXshelf = posXprod;
                                                    posYshelf = posYprod;
                                                    break;
                                                }
                                            }
                                        }
                                    }

                                    Instantiate(productPrefab, 
                                                new Vector3(posXshelf, 
                                                            1.0f, 
                                                            posYshelf) , 
                                                Quaternion.identity);
                                    Debug.Log("Current products instantiated");

                                    currentTime = DateTime.UtcNow;
                                    LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + 
                                                " - Current products instantiated") ;

                                    // Update user products
                                    User userScript = user.GetComponent<User>();
                                    userScript.productCollected.Clear();
                                    userScript.productList.Clear();
                                    foreach(string pN in data.productNames)
                                    {
                                        userScript.productList.Add(pN);
                                    }
                                    userScript.productCollCount = 0;
                                    userScript.productCount = data.productNames.Length;
                                }
                            }

                        }

                        // Update robot status
                        if (data.status == "Pause"){status = "Pause";}
                        else if (data.status == "Resume"){status = "Continue";}
                    }
                    catch (Exception e)
                    {
                        Debug.LogError("Error deserializing JSON: " + e.Message);

                        DateTime currentTime = DateTime.UtcNow;
                        LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + 
                                    " - Error JSON") ;
                    }
                }
                else
                {
                    Debug.Log("Client disconnected.");

                    DateTime currentTime = DateTime.UtcNow;
                    LogToFileApp(String.Format("{0:d/M/yyyy_HH:mm:ss}", currentTime) + 
                                " - Client disconnected") ;

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

    // Detects when the robot is colliding the floor
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

    // Save log files
    private void LogToFile(string logmsg)
    {
        using (StreamWriter writer = new StreamWriter(logFilePath, true))
        {
            writer.WriteLine(logmsg);
        }
    }

    private void LogToFileApp(string logmsg)
    {
        using (StreamWriter writer = new StreamWriter(logFilePathApp, true))
        {
            writer.WriteLine(logmsg);
        }
    }
}
