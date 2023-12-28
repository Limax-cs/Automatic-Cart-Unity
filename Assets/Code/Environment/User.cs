using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System;

public class User : MonoBehaviour
{
    //Controls
    public float horizontalMove;
    public float verticalMove;
    public float mouseX;
    public float mouseY;
    public CharacterController user;
    public Animator animator;
    private Vector3 userInput;

    //Camera
    public GameObject mainCamera;
    private Vector3 camForward;
    private Vector3 camRight;
    public float rotationConstant = 10;

    //Movement
    private Vector3 moveUser;
    private Vector3 userInputGrounded;
    public float userSpeed = 0.1f;
    public float JumpForce = 10;
    public float gravity = 9.81f;
    private float fallSpeed = 0;
    public float InAirFriction = 0.995f;
    public float InertiaContribution = 0.9f;

    // Status
    public List<string> productList;
    public List<string> productCollected;
    public int productCount = 0;
    public int productCollCount = 0;
    public TextMeshProUGUI statusUI;


    // Start is called before the first frame update
    void Start()
    {
        user = GetComponent<CharacterController>();
        productList = new List<string>();
        productCollected = new List<string>();
        mainCamera.transform.localEulerAngles = new Vector3(0,0,0);
    }

    // Update is called once per frame
    void Update()
    {
        // Control Detection
        horizontalMove = Input.GetAxis("Horizontal");
        verticalMove = Input.GetAxis("Vertical");
        mouseX = Input.GetAxis("Mouse X");
        mouseY = Input.GetAxis("Mouse Y");

        if (user.isGrounded)
        {
            userInput = new Vector3(horizontalMove, 0, verticalMove);
            userInput = Vector3.ClampMagnitude(userInput, 1)*userSpeed;
            userInputGrounded = userInput;
        }
        else
        {
            userInput = new Vector3(userInputGrounded.x + (1 - InertiaContribution) * horizontalMove, 0, userInputGrounded.z + (1 - InertiaContribution) * verticalMove);
            userInput = Vector3.ClampMagnitude(userInput, 1) * InAirFriction;
        }

        //Moviment del jugador segons la càmera
        CamDirection();
        moveUser = userInput.x * camRight + userInput.z * camForward;
        Vector3 forwardUser = user.transform.position + (userInput.z + userInput.x * userInput.x) * camForward;

        // Falling
        SetGravity();

        // Move the user
        user.Move(moveUser * Time.deltaTime);
        this.transform.Rotate(new Vector3(0,mouseX*400,0)*Time.deltaTime);

        // Move Camera
        Vector3 newCamAngles = mainCamera.transform.localEulerAngles + new Vector3(-mouseY*150*Time.deltaTime, 0 ,0);
        //Debug.Log(newCamAngles);
        if (newCamAngles[0]> 30 && newCamAngles[0] <= 180)
        {newCamAngles[0] = 30;}
        else if (newCamAngles[0] < 359 && newCamAngles[0] > 180)
        {newCamAngles[0] = 359;}
        newCamAngles[1] = 0.0f;
        newCamAngles[2] = 0.0f;
        mainCamera.transform.localEulerAngles = newCamAngles;

        // Animator
        animator.SetFloat("velocityFront", verticalMove*100);
        animator.SetFloat("velocityBack", -verticalMove*100);
        animator.SetFloat("velocitySide1", horizontalMove*100);
        animator.SetFloat("velocitySide2", -horizontalMove*100);


        // Update Products Collected
        //statusUI.text = "Products Collected: " + productCollCount + "/ " + productCount + " \n Products: " + string.Join(",", productList) + " \n Products Collected: " + string.Join(",", productCollected);
        statusUI.text = "Products Collected: " + productCollCount + "/ " + productCount;
        foreach(string item in productList)
        {
            statusUI.text = statusUI.text + " \n" + item;
            bool item_coll = false;

            foreach(string item_collected in productCollected)
            {
                if (item == item_collected)
                {
                    item_coll = true;
                }
            }

            if (item_coll)
            {
                statusUI.text = statusUI.text + " OK";
            }
        }
    }

    public void CamDirection()
    {
        camForward = mainCamera.transform.forward;
        camRight = mainCamera.transform.right;

        camForward.y = 0;
        camRight.y = 0;

        camForward = camForward.normalized;
        camRight = camRight.normalized;
    }

    public void SetGravity()
    {

        if (user.isGrounded)
        {
            moveUser.y = 0;
            fallSpeed = -gravity * Time.deltaTime;
            moveUser.y = fallSpeed;
        }
        else
        {
            fallSpeed -= gravity * Time.deltaTime;
            moveUser.y = fallSpeed;
        }

        
    }

    public void OnTriggerEnter(Collider collider)
    {
        if (collider.CompareTag("Product"))
        {
            productCollected.Add(collider.name.Substring(0, collider.name.Length -7));
            productCollCount = productCollCount + 1;

            Debug.Log("User collide with " + collider.name.Substring(0, collider.name.Length -7));
            Destroy(collider.gameObject);
        }
    }
}
