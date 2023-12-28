using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CostumerGenerator : MonoBehaviour
{

    public GameObject entityPrefab;
    public bool generateCustomers = false;

    public GameObject[] doors;
    public List<GameObject> doorsEnable;

    public float accTime = 0;
    public float randTime;

    // Start is called before the first frame update
    void Start()
    {

        doors = GameObject.FindGameObjectsWithTag("Door");
        randTime = Random.Range(2,20);
    }

    // Update is called once per frame
    void Update()
    {
        if(generateCustomers)
        {
            accTime += Time.deltaTime;
            if (accTime > randTime)
            {
                doorsEnable = new List<GameObject>();
                foreach(GameObject go in doors)
                {
                    if (go.activeInHierarchy == true)
                    {
                        doorsEnable.Add(go);
                    }
                }
                Instantiate(entityPrefab, doorsEnable[0].transform.position, Quaternion.identity);
                randTime = Random.Range(2,15);
                accTime = 0;
            }
        }
    }
}
