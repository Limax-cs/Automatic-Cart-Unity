using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class CostumerBehaviour : MonoBehaviour
{

    public UnityEngine.AI.NavMeshAgent agent;
    public GameObject[] shelves3;
    public GameObject[] shelves4;
    public GameObject[] doors;
    public List<GameObject> goals;
    public float timeCounter = 0;
    public Animator animator;


    // Start is called before the first frame update
    void Start()
    {
        agent = GetComponent<UnityEngine.AI.NavMeshAgent>(); 
        shelves3 = GameObject.FindGameObjectsWithTag("shelve_shop3");
        shelves4 = GameObject.FindGameObjectsWithTag("shelve_shop4");
        doors = GameObject.FindGameObjectsWithTag("Door");
        animator.SetFloat("velocityFront", 3.0f);
        
        //agent.destination= goal.position; 

        // Select a random amount of shelves
        System.Random random = new System.Random();

        int itemsNum = random.Next(1, 10); 

        for (int i = 0; i < itemsNum; i++)
        {
            int randomIndex;
            if (shelves3.Length > 0)
            {
                randomIndex = random.Next(0, shelves3.Length);
                GameObject selectedItem = shelves3[randomIndex];
                goals.Add(selectedItem);
            }
            else if (shelves4.Length > 0)
            {
                randomIndex = random.Next(0, shelves4.Length);
                GameObject selectedItem = shelves4[randomIndex];
                goals.Add(selectedItem);
            }
        }

        int randomDoor = random.Next(0, doors.Length);
        GameObject selectedDoor = doors[randomDoor];
        goals.Add(selectedDoor);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (goals.Count > 0)
        {
            //animator.SetFloat("velocityFront", 3.0f);
            agent.destination = goals[0].transform.position;

            if (goals.Count == 1 && Vector3.Distance(this.transform.position, goals[0].transform.position) < 3)
            {
                goals.RemoveAt(0);
            }
            else if(Vector3.Distance(this.transform.position, goals[0].transform.position) < 2)
            {
                timeCounter += Time.deltaTime;
                if (timeCounter > 3)
                {
                    goals.RemoveAt(0);
                    timeCounter = 0;
                    
                }

            }

            if(Vector3.Distance(this.transform.position, goals[0].transform.position) < 2.0f)
            {
                animator.SetFloat("velocityFront", 0.0f);
            }
            else
            {
                animator.SetFloat("velocityFront", 3.0f);
            }
        }
        else
        {
            animator.SetFloat("velocityFront", 3.0f);
            Destroy(gameObject);

        }
        
    }
}
