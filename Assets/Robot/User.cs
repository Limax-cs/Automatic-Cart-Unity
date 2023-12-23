using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class User : MonoBehaviour
{
    //Controls
    public float horizontalMove;
    public float verticalMove;
    public float mouseX;
    public float mouseY;
    public CharacterController user;
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


    // Start is called before the first frame update
    void Start()
    {
        user = GetComponent<CharacterController>();
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
        this.transform.Rotate(new Vector3(0,mouseX*1000,0)*Time.deltaTime);
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
}
