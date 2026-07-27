using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class RoomIndex
{

	public float LevelOneHeight = 1.0f; // Height of the first floor in Unity units
	public float LevelTwoHeight = 1.0f + 10f; // Height of the second floor in Unity units


	// Walls - inches
	private static int _floorID  = 1;
	private static int _wallID   = 1;
	private static int _vertexID = 1;



	public List<string> FloorType = new List<string>() { "LVP", "Carpet", "Tile", "Rubber" };
	public static List<FloorSection> FloorSectionList = new List<FloorSection>();

	// ----- ----- ----- ----- ----- ----- ----- FloorSection Class ----- ----- ----- ----- ----- ----- ----- 

	public class FloorSection
	{

		public  int     ID;
		public  string  Name;
		public  int     Level;
		public  Vector3 SpawnPos;
		public  float   Width;  // North - South 
		public  float   Length; // East - West
		public  float   Sqft;
		public  string  FloorType;
		public  bool	  IsActive;

		// Always returns the real runtime class name (e.g. "Rm_Bath")
		public string ClassRef => GetType().Name;

		private const float SqftConversion = 144f; // 12x12 inches

		// -----  Constructor ----- 
		public FloorSection(string name, int level, Vector3 spawnPos, float width, float length, string floorType, bool isActive = true)
		{
			ID        = _floorID;
			_floorID = _floorID + 1;

			Name      = name;
			Level     = level;
			SpawnPos  = spawnPos;
			Width     = width;
			Length    = length;
			Sqft      = length * width / SqftConversion;
			FloorType = floorType;
			IsActive  = isActive;

			FloorSectionList.Add(this);
		}
	}
	


	// ----- ----- ----- ----- ----- ----- ----- Initializer ----- ----- ----- ----- ----- ----- -----

	public static FloorSection Rm_EntryWay			 = new FloorSection("EntryWay",					 	1, new Vector3(0,0,0),     43f, 40.25f, "LVP");
	public static FloorSection Rm_EntryCloset		 = new FloorSection("EntryCloset",			 	1, new Vector3(0,0,0),     57f,    35f, "LVP");
	public static FloorSection Rm_Hallway				 = new FloorSection("Hallway",					  1, new Vector3(0,0,0),    229f,    36f, "LVP");
	public static FloorSection Rm_Bath					 = new FloorSection("Bathroom",					  1, new Vector3(0,0,0),  71.25f,    87f, "LVP");
	public static FloorSection Rm_Bed						 = new FloorSection("Bedroom",					  1, new Vector3(0,0,0),    157f,   148f, "LVP");
	public static FloorSection Rm_BedCloset			 = new FloorSection("BedroomCloset",		 	1, new Vector3(0,0,0),     71f,    78f, "LVP");
	public static FloorSection Rm_StoreEntry		 = new FloorSection("StoreroomEntry",		  1, new Vector3(0,0,0),   65.5f,   103f, "Carpet");
	public static FloorSection Rm_Store					 = new FloorSection("Storeroom",				  1, new Vector3(0,0,0),    120f, 127.5f, "Carpet");
	public static FloorSection Rm_Kitchen				 = new FloorSection("Kitchen",					  1, new Vector3(0,0,0),    114f,  90.5f, "LVP");
	public static FloorSection Rm_Laundry				 = new FloorSection("Laundry",					  1, new Vector3(0,0,0),  31.25f,  89.5f, "LVP");
	public static FloorSection Rm_Living				 = new FloorSection("LivingRoom",				  1, new Vector3(0,0,0), 151.75f,   152f, "LVP");
	public static FloorSection Rm_Dining				 = new FloorSection("DiningRoom",				  1, new Vector3(0,0,0), 125.75f, 95.25f, "LVP");
	public static FloorSection Rm_DeckCovered		 = new FloorSection("DeckCovered",			 	1, new Vector3(0,0,0),    158f,    40f, "Rubber");
	public static FloorSection Rm_DeckUncovered	 = new FloorSection("DeckUncovered",		  1, new Vector3(0,0,0),    185f,    28f, "Rubber");
	public static FloorSection Rm_Loft					 = new FloorSection("Loft",								2, new Vector3(0,0,0),    183f,   137f, "Carpet");
	public static FloorSection Rm_UpperBed			 = new FloorSection("UpperBedroom",				2, new Vector3(0,0,0),    165f,    76f, "Carpet");
	public static FloorSection Rm_UpperBedEntry	 = new FloorSection("UpperBedroomEntry",	2, new Vector3(0,0,0),     87f, 54.75f, "Carpet");
	public static FloorSection Rm_UpperBedCloset = new FloorSection("UpperBedroomCloset", 2, new Vector3(0,0,0),  23.75f,    76f, "Carpet");



	public void Start()
	{
		Calculate_Vertices();
	}



	// ----- ----- ----- ----- ----- ----- ----- Rm_Vertex Class ----- ----- ----- ----- ----- ----- ----- 
	public class Rm_Vertex
	{
		private int ID;
		public string Name;
		public Vector3 Vertex;

		// -----  Constructor ----- 
		public Rm_Vertex(string name, Vector3 vertex)
		{
			ID = _vertexID;
			_vertexID = _vertexID + 1;
			Name = name;
			Vertex = vertex;
		}
	}

	public Rm_Vertex GenerateVertex(string name, Vector3 position)
	{
		Rm_Vertex result = new Rm_Vertex(name, position);

		return result;
	}



	public void Calculate_Vertices()
	{

		// 1 Rm_Living_Vertices;
		Vector3 Rm_Living_SpawnPos = new Vector3(0, LevelOneHeight, 0); // Starting Reference (Furthest North-West Corner of the House)

		Vector3 Rm_Living_TopLeft     = new Vector3(Rm_Living_SpawnPos.x,										Rm_Living_SpawnPos.y, Rm_Living_SpawnPos.z + Rm_Living.Length);
		Vector3 Rm_Living_TopRight    = new Vector3(Rm_Living_SpawnPos.x + Rm_Living.Width, Rm_Living_SpawnPos.y, Rm_Living_SpawnPos.z + Rm_Living.Length);
		Vector3 Rm_Living_BottomLeft  = new Vector3(Rm_Living_SpawnPos.x,										Rm_Living_SpawnPos.y, Rm_Living_SpawnPos.z);
		Vector3 Rm_Living_BottomRight = new Vector3(Rm_Living_SpawnPos.x + Rm_Living.Width, Rm_Living_SpawnPos.y, Rm_Living_SpawnPos.z);

		List<Vector3> Rm_Living_Vertices = new List<Vector3> { Rm_Living_TopLeft, Rm_Living_TopRight, Rm_Living_BottomLeft, Rm_Living_BottomRight };

		List<Rm_Vertex> Rm_Living_VertexList = new List<Rm_Vertex> {
			GenerateVertex(nameof(Rm_Living_TopLeft), Rm_Living_TopLeft),
			GenerateVertex(nameof(Rm_Living_TopRight), Rm_Living_TopRight),
			GenerateVertex(nameof(Rm_Living_BottomLeft), Rm_Living_BottomLeft),
			GenerateVertex(nameof(Rm_Living_BottomRight), Rm_Living_BottomRight) 
		};
		List<List<Rm_Vertex>> Rm_All_Vertex_List = new List<List<Rm_Vertex>> { Rm_Living_VertexList };



		// 2 Rm_Dining_Vertices()
		Vector3 Rm_Dining_SpawnPos = Rm_Living_TopLeft;

		Vector3 Rm_Dining_TopLeft			= new Vector3(Rm_Dining_SpawnPos.x,										Rm_Dining_SpawnPos.y, Rm_Dining_SpawnPos.z + Rm_Dining.Length);
		Vector3 Rm_Dining_TopRight		= new Vector3(Rm_Dining_SpawnPos.x + Rm_Dining.Width, Rm_Dining_SpawnPos.y, Rm_Dining_SpawnPos.z + Rm_Dining.Length);
		Vector3 Rm_Dining_BottomLeft	= new Vector3(Rm_Dining_SpawnPos.x,										Rm_Dining_SpawnPos.y, Rm_Dining_SpawnPos.z);
		Vector3 Rm_Dining_BottomRight	= new Vector3(Rm_Dining_SpawnPos.x + Rm_Dining.Width, Rm_Dining_SpawnPos.y, Rm_Dining_SpawnPos.z);

		List<Vector3> Rm_Dining_Vertices = new List<Vector3>();
		Rm_Dining_Vertices.AddRange(new Vector3[] { Rm_Dining_TopLeft, Rm_Dining_TopRight, Rm_Dining_BottomLeft, Rm_Dining_BottomRight });



		// 3 Rm_Hallway_Vertices()
		Vector3 Rm_Hallway_SpawnPos		 = new Vector3(Rm_Living_TopRight.x, LevelOneHeight, Rm_Living_TopRight.z - Rm_Hallway.Length);

		Vector3 Rm_Hallway_TopLeft		 = new Vector3(Rm_Hallway_SpawnPos.x,										 Rm_Hallway_SpawnPos.y, Rm_Hallway_SpawnPos.z + Rm_Hallway.Length);
		Vector3 Rm_Hallway_TopRight		 = new Vector3(Rm_Hallway_SpawnPos.x + Rm_Hallway.Width, Rm_Hallway_SpawnPos.y, Rm_Hallway_SpawnPos.z + Rm_Hallway.Length);
		Vector3 Rm_Hallway_BottomLeft	 = new Vector3(Rm_Hallway_SpawnPos.x,										 Rm_Hallway_SpawnPos.y, Rm_Hallway_SpawnPos.z);
		Vector3 Rm_Hallway_BottomRight = new Vector3(Rm_Hallway_SpawnPos.x + Rm_Hallway.Width, Rm_Hallway_SpawnPos.y, Rm_Hallway_SpawnPos.z);

		List<Vector3> Rm_Hallway_Vertices = new List<Vector3>();
		Rm_Hallway_Vertices.AddRange(new Vector3[] { Rm_Hallway_TopLeft, Rm_Hallway_TopRight, Rm_Hallway_BottomLeft, Rm_Hallway_BottomRight });



		// 4 Rm_Bed_Vertices()
		Vector3 Rm_Bed_SpawnPos				 = new Vector3(Rm_Living_BottomRight.x, LevelOneHeight, Rm_Living_BottomRight.z - Rm_Hallway.Length);

		Vector3 Rm_Bed_TopLeft		 = new Vector3(Rm_Bed_SpawnPos.x,								 Rm_Bed_SpawnPos.y, Rm_Bed_SpawnPos.z + Rm_Bed.Length);
		Vector3 Rm_Bed_TopRight		 = new Vector3(Rm_Bed_SpawnPos.x + Rm_Bed.Width, Rm_Bed_SpawnPos.y, Rm_Bed_SpawnPos.z + Rm_Bed.Length);
		Vector3 Rm_Bed_BottomLeft	 = new Vector3(Rm_Bed_SpawnPos.x,								 Rm_Bed_SpawnPos.y, Rm_Bed_SpawnPos.z);
		Vector3 Rm_Bed_BottomRight = new Vector3(Rm_Bed_SpawnPos.x + Rm_Bed.Width, Rm_Bed_SpawnPos.y, Rm_Bed_SpawnPos.z);

		List<Vector3> Rm_Bed_Vertices = new List<Vector3>();
		Rm_Bed_Vertices.AddRange(new Vector3[] { Rm_Bed_TopLeft, Rm_Bed_TopRight, Rm_Bed_BottomLeft, Rm_Bed_BottomRight });



		// 5 Rm_Kitchen_Vertices()
		Vector3 Rm_Kitchen_SpawnPos		 = new Vector3(Rm_Dining_TopRight.x, LevelOneHeight, Rm_Dining_TopRight.z - Rm_Kitchen.Length);

		Vector3 Rm_Kitchen_TopLeft		 = new Vector3(Rm_Kitchen_SpawnPos.x,										 Rm_Kitchen_SpawnPos.y, Rm_Kitchen_SpawnPos.z + Rm_Kitchen.Length);
		Vector3 Rm_Kitchen_TopRight		 = new Vector3(Rm_Kitchen_SpawnPos.x + Rm_Kitchen.Width, Rm_Kitchen_SpawnPos.y, Rm_Kitchen_SpawnPos.z + Rm_Kitchen.Length);
		Vector3 Rm_Kitchen_BottomLeft	 = new Vector3(Rm_Kitchen_SpawnPos.x,										 Rm_Kitchen_SpawnPos.y, Rm_Kitchen_SpawnPos.z);
		Vector3 Rm_Kitchen_BottomRight = new Vector3(Rm_Kitchen_SpawnPos.x + Rm_Kitchen.Width, Rm_Kitchen_SpawnPos.y, Rm_Kitchen_SpawnPos.z);

		List<Vector3> Rm_Kitchen_Vertices = new List<Vector3>();
		Rm_Kitchen_Vertices.AddRange(new Vector3[] { Rm_Kitchen_TopLeft, Rm_Kitchen_TopRight, Rm_Kitchen_BottomLeft, Rm_Kitchen_BottomRight });



		// 6 Rm_Laundry_Vertices()
		Vector3 Rm_Laundry_SpawnPos		 = new Vector3(Rm_Kitchen_TopRight.x, LevelOneHeight, Rm_Kitchen_TopRight.z - Rm_Laundry.Length);

		Vector3 Rm_Laundry_TopLeft		 = new Vector3(Rm_Laundry_SpawnPos.x,										 Rm_Laundry_SpawnPos.y, Rm_Laundry_SpawnPos.z + Rm_Laundry.Length);
		Vector3 Rm_Laundry_TopRight		 = new Vector3(Rm_Laundry_SpawnPos.x + Rm_Laundry.Width, Rm_Laundry_SpawnPos.y, Rm_Laundry_SpawnPos.z + Rm_Laundry.Length);
		Vector3 Rm_Laundry_BottomLeft	 = new Vector3(Rm_Laundry_SpawnPos.x,										 Rm_Laundry_SpawnPos.y, Rm_Laundry_SpawnPos.z);
		Vector3 Rm_Laundry_BottomRight = new Vector3(Rm_Laundry_SpawnPos.x + Rm_Laundry.Width, Rm_Laundry_SpawnPos.y, Rm_Laundry_SpawnPos.z);

		List<Vector3> Rm_Laundry_Vertices = new List<Vector3>();
		Rm_Laundry_Vertices.AddRange(new Vector3[] { Rm_Laundry_TopLeft, Rm_Laundry_TopRight, Rm_Laundry_BottomLeft, Rm_Laundry_BottomRight });



		// 7 Rm_EntryWay_Vertices()
		Vector3 Rm_EntryWay_SpawnPos		= new Vector3(Rm_Laundry_BottomRight.x, LevelOneHeight, Rm_Laundry_BottomRight.z);

		Vector3 Rm_EntryWay_TopLeft			= new Vector3(Rm_EntryWay_SpawnPos.x,											Rm_EntryWay_SpawnPos.y, Rm_EntryWay_SpawnPos.z + Rm_EntryWay.Length);
		Vector3 Rm_EntryWay_TopRight		= new Vector3(Rm_EntryWay_SpawnPos.x + Rm_EntryWay.Width, Rm_EntryWay_SpawnPos.y, Rm_EntryWay_SpawnPos.z + Rm_EntryWay.Length);
		Vector3 Rm_EntryWay_BottomLeft	= new Vector3(Rm_EntryWay_SpawnPos.x,											Rm_EntryWay_SpawnPos.y, Rm_EntryWay_SpawnPos.z);
		Vector3 Rm_EntryWay_BottomRight = new Vector3(Rm_EntryWay_SpawnPos.x + Rm_EntryWay.Width, Rm_EntryWay_SpawnPos.y, Rm_EntryWay_SpawnPos.z);

		List<Vector3> Rm_EntryWay_Vertices = new List<Vector3>();
		Rm_EntryWay_Vertices.AddRange(new Vector3[] { Rm_EntryWay_TopLeft, Rm_EntryWay_TopRight, Rm_EntryWay_BottomLeft, Rm_EntryWay_BottomRight });



		// 8 Rm_EntryWayCloset_Vertices()
		Vector3 Rm_EntryWayCloset_SpawnPos		= new Vector3(Rm_EntryWay_TopRight.x, LevelOneHeight, Rm_EntryWay_TopRight.z - Rm_EntryCloset.Length);

		Vector3 Rm_EntryWayCloset_TopLeft			= new Vector3(Rm_EntryWayCloset_SpawnPos.x,												 Rm_EntryWayCloset_SpawnPos.y, Rm_EntryWayCloset_SpawnPos.z + Rm_EntryCloset.Length);
		Vector3 Rm_EntryWayCloset_TopRight		= new Vector3(Rm_EntryWayCloset_SpawnPos.x + Rm_EntryCloset.Width, Rm_EntryWayCloset_SpawnPos.y, Rm_EntryWayCloset_SpawnPos.z + Rm_EntryCloset.Length);
		Vector3 Rm_EntryWayCloset_BottomLeft	= new Vector3(Rm_EntryWayCloset_SpawnPos.x,												 Rm_EntryWayCloset_SpawnPos.y, Rm_EntryWayCloset_SpawnPos.z);
		Vector3 Rm_EntryWayCloset_BottomRight = new Vector3(Rm_EntryWayCloset_SpawnPos.x + Rm_EntryCloset.Width, Rm_EntryWayCloset_SpawnPos.y, Rm_EntryWayCloset_SpawnPos.z);

		List<Vector3> Rm_EntryWayCloset_Vertices = new List<Vector3>();
		Rm_EntryWayCloset_Vertices.AddRange(new Vector3[] { Rm_EntryWayCloset_TopLeft, Rm_EntryWayCloset_TopRight, Rm_EntryWayCloset_BottomLeft, Rm_EntryWayCloset_BottomRight });



		// 9 Rm_Bath_Vertices()
		Vector3 Rm_Bath_SpawnPos		= new Vector3(Rm_Hallway_BottomRight.x - Rm_Bath.Width, LevelOneHeight, Rm_Hallway_BottomRight.z - Rm_Bath.Length);

		Vector3 Rm_Bath_TopLeft			= new Vector3(Rm_Bath_SpawnPos.x,									Rm_Bath_SpawnPos.y, Rm_Bath_SpawnPos.z + Rm_Bath.Length);
		Vector3 Rm_Bath_TopRight		= new Vector3(Rm_Bath_SpawnPos.x + Rm_Bath.Width, Rm_Bath_SpawnPos.y, Rm_Bath_SpawnPos.z + Rm_Bath.Length);
		Vector3 Rm_Bath_BottomLeft	= new Vector3(Rm_Bath_SpawnPos.x,									Rm_Bath_SpawnPos.y, Rm_Bath_SpawnPos.z);
		Vector3 Rm_Bath_BottomRight = new Vector3(Rm_Bath_SpawnPos.x + Rm_Bath.Width, Rm_Bath_SpawnPos.y, Rm_Bath_SpawnPos.z);

		List<Vector3> Rm_Bath_Vertices = new List<Vector3>();
		Rm_Bath_Vertices.AddRange(new Vector3[] { Rm_Bath_TopLeft, Rm_Bath_TopRight, Rm_Bath_BottomLeft, Rm_Bath_BottomRight });



		// 10 Rm_BedCloset_Vertices()
		Vector3 Rm_BedCloset_SpawnPos			= new Vector3(Rm_Bath_BottomRight.x - Rm_BedCloset.Width, LevelOneHeight, Rm_Bath_BottomRight.z - Rm_BedCloset.Length);

		Vector3 Rm_BedCloset_TopLeft			= new Vector3(Rm_BedCloset_SpawnPos.x,											Rm_BedCloset_SpawnPos.y, Rm_BedCloset_SpawnPos.z + Rm_BedCloset.Length);
		Vector3 Rm_BedCloset_TopRight			= new Vector3(Rm_BedCloset_SpawnPos.x + Rm_BedCloset.Width, Rm_BedCloset_SpawnPos.y, Rm_BedCloset_SpawnPos.z + Rm_BedCloset.Length);
		Vector3 Rm_BedCloset_BottomLeft		= new Vector3(Rm_BedCloset_SpawnPos.x,											Rm_BedCloset_SpawnPos.y, Rm_BedCloset_SpawnPos.z);
		Vector3 Rm_BedCloset_BottomRight	= new Vector3(Rm_BedCloset_SpawnPos.x + Rm_BedCloset.Width, Rm_BedCloset_SpawnPos.y, Rm_BedCloset_SpawnPos.z);

		List<Vector3> Rm_BedCloset_Vertices = new List<Vector3>();
		Rm_BedCloset_Vertices.AddRange(new Vector3[] { Rm_BedCloset_TopLeft, Rm_BedCloset_TopRight, Rm_BedCloset_BottomLeft, Rm_BedCloset_BottomRight });



		// 11 Rm_StoreEntry_Vertices()
		Vector3 Rm_StoreEntry_SpawnPos		= new Vector3(Rm_BedCloset_BottomRight.x - Rm_StoreEntry.Width, LevelOneHeight, Rm_BedCloset_BottomRight.z - Rm_StoreEntry.Length);

		Vector3 Rm_StoreEntry_TopLeft			= new Vector3(Rm_StoreEntry_SpawnPos.x,												Rm_StoreEntry_SpawnPos.y, Rm_StoreEntry_SpawnPos.z + Rm_StoreEntry.Length);
		Vector3 Rm_StoreEntry_TopRight		= new Vector3(Rm_StoreEntry_SpawnPos.x + Rm_StoreEntry.Width, Rm_StoreEntry_SpawnPos.y, Rm_StoreEntry_SpawnPos.z + Rm_StoreEntry.Length);
		Vector3 Rm_StoreEntry_BottomLeft	= new Vector3(Rm_StoreEntry_SpawnPos.x,											  Rm_StoreEntry_SpawnPos.y, Rm_StoreEntry_SpawnPos.z);
		Vector3 Rm_StoreEntry_BottomRight	= new Vector3(Rm_StoreEntry_SpawnPos.x + Rm_StoreEntry.Width, Rm_StoreEntry_SpawnPos.y, Rm_StoreEntry_SpawnPos.z);

		List<Vector3> Rm_StoreEntry_Vertices = new List<Vector3>();
		Rm_StoreEntry_Vertices.AddRange(new Vector3[] { Rm_StoreEntry_TopLeft, Rm_StoreEntry_TopRight, Rm_StoreEntry_BottomLeft, Rm_StoreEntry_BottomRight });



		// 12 Rm_Store_Vertices()
		Vector3 Rm_Store_SpawnPos		 = new Vector3(Rm_StoreEntry_BottomLeft.x - Rm_Store.Width, LevelOneHeight, Rm_StoreEntry_BottomLeft.z);

		Vector3 Rm_Store_TopLeft		 = new Vector3(Rm_Store_SpawnPos.x,									 Rm_Store_SpawnPos.y, Rm_Store_SpawnPos.z + Rm_Store.Length);
		Vector3 Rm_Store_TopRight		 = new Vector3(Rm_Store_SpawnPos.x + Rm_Store.Width, Rm_Store_SpawnPos.y, Rm_Store_SpawnPos.z + Rm_Store.Length);
		Vector3 Rm_Store_BottomLeft	 = new Vector3(Rm_Store_SpawnPos.x,									 Rm_Store_SpawnPos.y, Rm_Store_SpawnPos.z);
		Vector3 Rm_Store_BottomRight = new Vector3(Rm_Store_SpawnPos.x + Rm_Store.Width, Rm_Store_SpawnPos.y, Rm_Store_SpawnPos.z);

		List<Vector3> Rm_Store_Vertices = new List<Vector3>();
		Rm_Store_Vertices.AddRange(new Vector3[] { Rm_Store_TopLeft, Rm_Store_TopRight, Rm_Store_BottomLeft, Rm_Store_BottomRight });



		// 13 Rm_DeckCovered_Vertices()
		Vector3 Rm_DeckCovered_SpawnPos			= new Vector3(Rm_Living_BottomLeft.x, LevelOneHeight, Rm_Living_BottomLeft.z - Rm_DeckCovered.Length);

		Vector3 Rm_DeckCovered_TopLeft			= new Vector3(Rm_DeckCovered_SpawnPos.x,												Rm_DeckCovered_SpawnPos.y, Rm_DeckCovered_SpawnPos.z + Rm_DeckCovered.Length);
		Vector3	Rm_DeckCovered_TopRight			= new Vector3(Rm_DeckCovered_SpawnPos.x + Rm_DeckCovered.Width, Rm_DeckCovered_SpawnPos.y, Rm_DeckCovered_SpawnPos.z + Rm_DeckCovered.Length);
		Vector3 Rm_DeckCovered_BottomLeft		= new Vector3(Rm_DeckCovered_SpawnPos.x,												Rm_DeckCovered_SpawnPos.y, Rm_DeckCovered_SpawnPos.z);
		Vector3 Rm_DeckCovered_BottomRight	= new Vector3(Rm_DeckCovered_SpawnPos.x + Rm_DeckCovered.Width, Rm_DeckCovered_SpawnPos.y, Rm_DeckCovered_SpawnPos.z);

		List<Vector3> Rm_DeckCovered_Vertices = new List<Vector3>();
		Rm_DeckCovered_Vertices.AddRange(new Vector3[] { Rm_DeckCovered_TopLeft, Rm_DeckCovered_TopRight, Rm_DeckCovered_BottomLeft, Rm_DeckCovered_BottomRight });



		// 14 Rm_DeckUncovered_Vertices()
		Vector3 Rm_DeckUncovered_SpawnPos		 = new Vector3(Rm_DeckCovered_SpawnPos.x, LevelOneHeight, Rm_DeckCovered_SpawnPos.z - Rm_DeckUncovered.Length);

		Vector3 Rm_DeckUncovered_TopLeft		 = new Vector3(Rm_DeckUncovered_SpawnPos.x,													 Rm_DeckUncovered_SpawnPos.y, Rm_DeckUncovered_SpawnPos.z + Rm_DeckUncovered.Length);
		Vector3	Rm_DeckUncovered_TopRight		 = new Vector3(Rm_DeckUncovered_SpawnPos.x + Rm_DeckUncovered.Width, Rm_DeckUncovered_SpawnPos.y, Rm_DeckUncovered_SpawnPos.z + Rm_DeckUncovered.Length);
		Vector3 Rm_DeckUncovered_BottomLeft	 = new Vector3(Rm_DeckUncovered_SpawnPos.x,													 Rm_DeckUncovered_SpawnPos.y, Rm_DeckUncovered_SpawnPos.z);
		Vector3 Rm_DeckUncovered_BottomRight = new Vector3(Rm_DeckUncovered_SpawnPos.x + Rm_DeckUncovered.Width, Rm_DeckUncovered_SpawnPos.y, Rm_DeckUncovered_SpawnPos.z);

		List<Vector3> Rm_DeckUncovered_Vertices = new List<Vector3>();
		Rm_DeckUncovered_Vertices.AddRange(new Vector3[] { Rm_DeckUncovered_TopLeft, Rm_DeckUncovered_TopRight, Rm_DeckUncovered_BottomLeft, Rm_DeckUncovered_BottomRight });



		// 15 Rm_Loft_Vertices()
		Vector3 Rm_Loft_SpawnPos		= new Vector3(Rm_Dining_TopLeft.x, LevelTwoHeight, Rm_Dining_TopLeft.z - Rm_Loft.Length);			// Loft is on the second floor, so we add height

		Vector3 Rm_Loft_TopLeft			= new Vector3(Rm_Loft_SpawnPos.x,									Rm_Loft_SpawnPos.y, Rm_Loft_SpawnPos.z + Rm_Loft.Length);
		Vector3 Rm_Loft_TopRight		= new Vector3(Rm_Loft_SpawnPos.x + Rm_Loft.Width, Rm_Loft_SpawnPos.y, Rm_Loft_SpawnPos.z + Rm_Loft.Length);
		Vector3 Rm_Loft_BottomLeft  = new Vector3(Rm_Loft_SpawnPos.x,									Rm_Loft_SpawnPos.y, Rm_Loft_SpawnPos.z);
		Vector3 Rm_Loft_BottomRight = new Vector3(Rm_Loft_SpawnPos.x + Rm_Loft.Width, Rm_Loft_SpawnPos.y, Rm_Loft_SpawnPos.z);

		List<Vector3> Rm_Loft_Vertices = new List<Vector3>();
		Rm_Loft_Vertices.AddRange(new Vector3[] { Rm_Loft_TopLeft, Rm_Loft_TopRight, Rm_Loft_BottomLeft, Rm_Loft_BottomRight });



		// 16 Rm_UpperBedEntry_Vertices()
		Vector3 Rm_UpperBedEntry_SpawnPos		 = new Vector3(Rm_Loft_TopRight.x, LevelTwoHeight, Rm_Loft_TopRight.z - Rm_UpperBedEntry.Length);

		Vector3 Rm_UpperBedEntry_TopLeft		 = new Vector3(Rm_UpperBedEntry_SpawnPos.x,													 Rm_UpperBedEntry_SpawnPos.y, Rm_UpperBedEntry_SpawnPos.z + Rm_UpperBedEntry.Length);
		Vector3 Rm_UpperBedEntry_TopRight		 = new Vector3(Rm_UpperBedEntry_SpawnPos.x + Rm_UpperBedEntry.Width, Rm_UpperBedEntry_SpawnPos.y, Rm_UpperBedEntry_SpawnPos.z + Rm_UpperBedEntry.Length);
		Vector3 Rm_UpperBedEntry_BottomLeft  = new Vector3(Rm_UpperBedEntry_SpawnPos.x,													 Rm_UpperBedEntry_SpawnPos.y, Rm_UpperBedEntry_SpawnPos.z);
		Vector3 Rm_UpperBedEntry_BottomRight = new Vector3(Rm_UpperBedEntry_SpawnPos.x + Rm_UpperBedEntry.Width, Rm_UpperBedEntry_SpawnPos.y, Rm_UpperBedEntry_SpawnPos.z);

		List<Vector3> Rm_UpperBedEntry_Vertices = new List<Vector3>();
		Rm_UpperBedEntry_Vertices.AddRange(new Vector3[] { Rm_UpperBedEntry_TopLeft, Rm_UpperBedEntry_TopRight, Rm_UpperBedEntry_BottomLeft, Rm_UpperBedEntry_BottomRight });



		// 17 Rm_UpperBed_Vertices()
		Vector3 Rm_UpperBed_SpawnPos		= new Vector3(Rm_UpperBedEntry_BottomLeft.x, LevelTwoHeight, Rm_UpperBedEntry_BottomLeft.z - Rm_UpperBed.Length);

		Vector3 Rm_UpperBed_TopLeft			= new Vector3(Rm_UpperBed_SpawnPos.x,											Rm_UpperBed_SpawnPos.y, Rm_UpperBed_SpawnPos.z + Rm_UpperBed.Length);
		Vector3 Rm_UpperBed_TopRight		= new Vector3(Rm_UpperBed_SpawnPos.x + Rm_UpperBed.Width, Rm_UpperBed_SpawnPos.y, Rm_UpperBed_SpawnPos.z + Rm_UpperBed.Length);
		Vector3 Rm_UpperBed_BottomLeft	= new Vector3(Rm_UpperBed_SpawnPos.x,											Rm_UpperBed_SpawnPos.y, Rm_UpperBed_SpawnPos.z);
		Vector3 Rm_UpperBed_BottomRight = new Vector3(Rm_UpperBed_SpawnPos.x + Rm_UpperBed.Width, Rm_UpperBed_SpawnPos.y, Rm_UpperBed_SpawnPos.z);

		List<Vector3> Rm_UpperBed_Vertices = new List<Vector3>();
		Rm_UpperBed_Vertices.AddRange(new Vector3[] { Rm_UpperBed_TopLeft, Rm_UpperBed_TopRight, Rm_UpperBed_BottomLeft, Rm_UpperBed_BottomRight });



		// 18 Rm_UpperBedCloset_Vertices()
		Vector3 Rm_UpperBedCloset_SpawnPos		= new Vector3(Rm_UpperBed_BottomRight.x, LevelTwoHeight, Rm_UpperBed_BottomRight.z);

		Vector3 Rm_UpperBedCloset_TopLeft			= new Vector3(Rm_UpperBedCloset_SpawnPos.x,													 Rm_UpperBedCloset_SpawnPos.y, Rm_UpperBedCloset_SpawnPos.z + Rm_UpperBedCloset.Length);
		Vector3 Rm_UpperBedCloset_TopRight		= new Vector3(Rm_UpperBedCloset_SpawnPos.x + Rm_UpperBedCloset.Width, Rm_UpperBedCloset_SpawnPos.y, Rm_UpperBedCloset_SpawnPos.z + Rm_UpperBedCloset.Length);
		Vector3 Rm_UpperBedCloset_BottomLeft	= new Vector3(Rm_UpperBedCloset_SpawnPos.x,													 Rm_UpperBedCloset_SpawnPos.y, Rm_UpperBedCloset_SpawnPos.z);
		Vector3 Rm_UpperBedCloset_BottomRight	= new Vector3(Rm_UpperBedCloset_SpawnPos.x + Rm_UpperBedCloset.Width, Rm_UpperBedCloset_SpawnPos.y, Rm_UpperBedCloset_SpawnPos.z);

		List<Vector3> Rm_UpperBedCloset_Vertices = new List<Vector3>();
		Rm_UpperBedCloset_Vertices.AddRange(new Vector3[] { Rm_UpperBedCloset_TopLeft, Rm_UpperBedCloset_TopRight, Rm_UpperBedCloset_BottomLeft, Rm_UpperBedCloset_BottomRight });

		// 19 All_Room_Vertices()

		List<List<Vector3>> Rm_All_VerticesList = new List<List<Vector3>>();
		Rm_All_VerticesList.AddRange(new List<List<Vector3>> { 
			Rm_Living_Vertices, 
			Rm_Dining_Vertices, 
			Rm_Hallway_Vertices, 
			Rm_Bed_Vertices, 
			Rm_Kitchen_Vertices, 
			Rm_Laundry_Vertices, 
			Rm_EntryWay_Vertices, 
			Rm_EntryWayCloset_Vertices, 
			Rm_Bath_Vertices, 
			Rm_BedCloset_Vertices, 
			Rm_StoreEntry_Vertices, 
			Rm_Store_Vertices, 
			Rm_DeckCovered_Vertices, 
			Rm_DeckUncovered_Vertices, 
			Rm_Loft_Vertices, 
			Rm_UpperBedEntry_Vertices, 
			Rm_UpperBed_Vertices, 
			Rm_UpperBedCloset_Vertices 
		});




		UpdateSpawnPos("LivingRoom",				 Rm_Living_SpawnPos);
		UpdateSpawnPos("DiningRoom",				 Rm_Dining_SpawnPos);
		UpdateSpawnPos("Bedroom",						 Rm_Bed_SpawnPos);
		UpdateSpawnPos("Hallway",						 Rm_Hallway_SpawnPos);
		UpdateSpawnPos("Kitchen",						 Rm_Kitchen_SpawnPos);
		UpdateSpawnPos("Laundry",						 Rm_Laundry_SpawnPos);
		UpdateSpawnPos("EntryWay",					 Rm_EntryWay_SpawnPos);
		UpdateSpawnPos("EntryCloset",				 Rm_EntryWayCloset_SpawnPos);
		UpdateSpawnPos("Bathroom",					 Rm_Bath_SpawnPos);
		UpdateSpawnPos("BedroomCloset",			 Rm_BedCloset_SpawnPos);
		UpdateSpawnPos("StoreroomEntry",		 Rm_StoreEntry_SpawnPos);
		UpdateSpawnPos("Storeroom",					 Rm_Store_SpawnPos);
		UpdateSpawnPos("DeckCovered",				 Rm_DeckCovered_SpawnPos);
		UpdateSpawnPos("DeckUncovered",			 Rm_DeckUncovered_SpawnPos);
		UpdateSpawnPos("Loft",							 Rm_Loft_SpawnPos);
		UpdateSpawnPos("UpperBedroomEntry",	 Rm_UpperBedEntry_SpawnPos);
		UpdateSpawnPos("UpperBedroom",			 Rm_UpperBed_SpawnPos);
		UpdateSpawnPos("UpperBedroomCloset", Rm_UpperBedCloset_SpawnPos);




		ConsoleLogVertexList(Rm_All_Vertex_List);


	}


	void UpdateSpawnPos(string name, Vector3 newPos)
	{
		var section = RoomIndex.FloorSectionList.Find(s => s.Name == name);
		if (section != null)
			section.SpawnPos = newPos;
	}


	public void ConsoleLogVertexList(List<List<Rm_Vertex>> topList)
	{
		string consoleLog = "Room Vertices: \n";

		if (topList == null || topList.Count == 0){ 
			Debug.Log("No vertices to display.");
			return;
		}
		
		consoleLog += $"Room Vertices in {topList}: \n";
		foreach (List<Rm_Vertex> VertList in topList)
		{
			consoleLog += $"--{VertList} \n";
			foreach (Rm_Vertex vert in VertList)
			{
				consoleLog += 
					$"----{vert.Name} \n" +
					$"------ {vert.Vertex.x}, {vert.Vertex.y}, {vert.Vertex.z}\n";
			}
		}
		Debug.Log(consoleLog);
	}



	// ----- ----- ----- ----- ----- ----- ----- WallSection Class ----- ----- ----- ----- ----- ----- ----- 

	public enum WL_Direction
	{
		NorthSouth = 1,
		EastWest = 2
	}

	public static float WL_Thick = 5; // Wall thickness in inches

	public class WallSection
	{

		public int ID;
		public string Name;
		public int Level;
		public WL_Direction Direction;
		public Vector3 SpawnPos;
		public float Width;
		public float Length;
		public bool IsActive;



		// -----  Constructor ----- 
		public WallSection(string name, int level, WL_Direction direction, float width, float length, bool isActive)
		{
			ID = _wallID;
			_wallID = _wallID + 1;

			Name = name;
			Level = level;
			Direction = direction;
			Width = width;
			Length = length;
		}
	}


	// ----- ----- ----- ----- ----- ----- ----- Initializer ----- ----- ----- ----- ----- ----- -----

	WallSection WL_Entryway_EntrywayCloset = new WallSection("EntryClosetEast", 1,  WL_Direction.NorthSouth, WL_Thick, 5f, true);
	WallSection WL_EntryWay_Laundry				 = new WallSection("EntryClosetWest", 1, WL_Direction.NorthSouth, WL_Thick, 39.5f, true);






















	public class Hallway_Measurements
	{
		float length = 36;
		float width = 229;

		public float wall_hallway_south = 36;

		public float wall_hallway_north = 36; // No wall, simply end of hallway as it enters into living room

		public float wall_hallway_east = 229;
		public float wall_hallway_east_kitchen = 48; // 98-40 is the kitchen wall, minus where the living room starts
		public float wall_hallway_east_kitchenEntry_gap = 30;
		public float wall_hallway_east_laundryWall = 41;
		public float wall_hallway_east_entryGap = 43;
		public float wall_hallway_east_waterHeaterWall = 56.75f;

		public float wall_hallway_west = 229;
		public float wall_hallway_west_main = 120;
		public float wall_hallway_west_bedroomDoor_gap = 30;
		public float wall_hallway_west_bathroomDoor_gap = 30;
		public float wall_hallway_west_bedBathWall = 11.5f;
		public float wall_hallway_west_closetWall = 37.5f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Living Room Orginal Measurements ----- ----- ----- ----- ----- ----- ----- 
	public class Rm_LivingFullArea_Measurements // Room measured as two rooms. Original one room measurements saved here.
	{
		public float wall_livingroom_north = 253.25f;
		public float wall_livingroom_north_left_east = 58;
		public float wall_livingroom_north_fireplace = 41;
		public float wall_livingroom_north_left_west = 154.25f;

		public float wall_livingroom_south = 253.25f;
		public float wall_livingroom_south_kitchenShelf = 90.5f;
		public float wall_livingroom_south_hallwayPiece = 4.75f;
		public float wall_livingroom_south_hallway_gap = 36;
		public float wall_livingroom_south_mainroom = 122.5f;

		public float wall_livingroom_east = 125.75f;
		public float wall_livingroom_east_stairwell = 28.25f;
		public float wall_livingroom_east_stairFoot = 6;
		public float wall_livingroom_east_underStairs = 91.5f;

		public float wall_livingroom_west = 152;
		public float wall_livingroom_west_left = 28.5f;
		public float wall_livingroom_west_door = 95.5f;
		public float wall_livingroom_west_right = 28;

	}
	// ----- ----- ----- ----- ----- ----- ----- Living Room ----- ----- ----- ----- ----- ----- ----- 
	public class Rm_Living_Measurements
	{
		float length = 158.25f;
		float width = 152;

		public float wall_livingRoom_north = 158.25f;

		public float wall_livingRoom_south = 158.25f;
		public float wall_livingRoom_south_main = 122.25f;
		public float wall_livingRoom_south_hallwayGap = 36;

		public float wall_livingRoom_east = 152;

		public float wall_livingRoom_west = 152;
	}
	// ----- ----- ----- ----- ----- ----- ----- Dining Room ----- ----- ----- ----- ----- ----- ----- 
	public class Rm_Dining_Measurements
	{
		float length = 95.25f;
		float width = 125.75f;

		public float wall_diningRoom_north = 95.25f;

		public float wall_diningRoom_south = 95.25f;
		public float wall_diningRoom_south_bar = 90.5f;
		public float wall_diningRoom_south_post = 4.75f;

		public float wall_diningRoom_east = 125.75f;

		public float wall_diningRoom_west = 125.75f;

	}
	// ----- ----- ----- ----- ----- ----- ----- Entry Way ----- ----- ----- ----- ----- ----- ----- 
	public class EntryWay_Measurements
	{
		float length = 40.25f;
		float width = 43;

		public float wall_entryway_north = 40.25f;

		public float wall_entryway_south = 40.25f;
		public float wall_entryway_south_post = 5.25f;
		public float wall_entryway_south_closet = 30f;
		public float wall_entryway_south_wall = 5;

		public float wall_entryway_east = 43;
		public float wall_entryway_east_doorLeftSide = 3;
		public float wall_entryway_east_doorRightSide = 4;
		public float wall_entryway_frontDoor = 36;

		public float wall_entryway_west_gap = 43;
	}
	// ----- ----- ----- ----- ----- ----- ----- Entry Way Closet ----- ----- ----- ----- ----- ----- ----- 
	public class EntryCloset_Measurements
	{
		float length = 35;
		float width = 57;

		public float wall_entrywayCloset_south = 35;

		public float wall_entrywayCloset_north = 35;
		public float wall_entrywayCloset_north_post = 5;
		public float wall_entrywayCloset_north_gap = 30;

		public float wall_entrywayCloset_east = 57;

		public float wall_entrywayCloset_west = 57;
		public float wall_entrywayCloset_west_inner = 52;
		public float wall_entrywayCloset_west_post = 5;
	}
	// ----- ----- ----- ----- ----- ----- ----- Bathroom ----- ----- ----- ----- ----- ----- ----- 
	public class Bathroom_Measurements
	{
		float length = 63.75f;
		float width = 71.25f;

		public float wall_bathroom_north = 63.75f;
		public float wall_bathroom_north_right = 31.25f;
		public float wall_bathroom_north_doorFrame = 28;
		public float wall_bathroom_north_left = 7;

		public float wall_bathroom_south = 87f;
			public float wall_bathroom_left_south = 63.75f;
			public float wall_bathroom_shower_south = 63.75f;


		public float wall_bathroom_east = 71.25f;
		public float wall_bathroom_east_left = 3.25f;
		public float wall_bathroom_east_doorFrame = 30;
		public float wall_bathroom_east_right = 38;

		public float wall_bathroom_west = 71.25f;
		public float wall_bathroom_west_left = 12.25f;
		public float wall_bathroom_west_tub = 58.5f;
	}
	// ----- ----- ----- ----- ----- ----- ----- Bedroom ----- ----- ----- ----- ----- ----- ----- 
	public class Bedroom_Measurements
	{
		float length = 157;
		float width = 148;

		public float wall_bedroom_north = 157;

		public float wall_bedroom_south = 157;
		public float wall_bedroom_south_left = 31.5f;
		public float wall_bedroom_south_bathDoor = 28.25f;
		public float wall_bedroom_south_right = 62f;
		public float wall_bedroom_south_gap = 35.25f;

		public float wall_bedroom_east = 148;
		public float wall_bedroom_east_short = 3;
		public float wall_bedroom_east_doorFrame = 30;
		public float wall_bedroom_east_long = 125;

		public float wall_bedroom_west = 148;
	}
	// ----- ----- ----- ----- ----- ----- ----- BedroomCloset ----- ----- ----- ----- ----- ----- ----- 
	public class BedroomCloset_Measurements
	{
		float length = 78;
		float width = 71;

		public float wall_closet_north = 78;
		public float wall_closet_north_right = 21;
		public float wall_closet_north_gap = 35;
		public float wall_closet_north_left = 22;

		public float wall_closet_south = 78;

		public float wall_closet_east = 71;

		public float wall_closet_west = 71;
		public float wall_closet_west_left = 18;
		public float wall_closet_west_doorFrame = 29.75f;
		public float wall_closet_west_right = 23.12f;
	}
	// ----- ----- ----- ----- ----- ----- ----- StoreroomEntry ----- ----- ----- ----- ----- ----- ----- 
	public class StoreroomEntry_Measurements
	{
		float length = 103f;
		float width = 65.5f;

		public float wall_storeroom_south = 103f;

		public float wall_storeroom_east = 65.5f;
	}

		// ----- ----- ----- ----- ----- ----- ----- Storeroom ----- ----- ----- ----- ----- ----- ----- 
		public class Storeroom_Measurements
		{
			float length = 127.5f;
			float width = 120f;

			public float wall_storeroom_north = 127.5f;

			public float wall_storeroom_east = 120f;


		}
		// ----- ----- ----- ----- ----- ----- ----- Kitchen ----- ----- ----- ----- ----- ----- ----- 
		public class Kitchen_Measurements
		{
			float length = 90.5f;
			float width = 114f;

			public float wall_kitchen_north_kitchenShelf = 90.5f; // where living room wall meets kitchen wall

			public float wall_kitchen_south_hallway = 90.5f; // where laundry space starts

			public float wall_kitchen_east = 114; // where laundry space starts

			public float wall_kitchen_west = 114; // where laundry space starts
		}
		// ----- ----- ----- ----- ----- ----- ----- Laundry Space ----- ----- ----- ----- ----- ----- ----- 
		public class Laundry_Measurements
		{
			float length = 89.5f;
			float width = 31.25f;

			public float wall_laundry_north = 89.5f;

			public float wall_laundry_south = 89.5f;
			public float wall_laundry_south_right = 30f;
			public float wall_laundry_south_gap = 59.5f;

			public float wall_laundry_east = 31.25f;

			public float wall_laundry_west = 31.25f;
		}
		// ----- ----- ----- ----- ----- ----- ----- Loft ----- ----- ----- ----- ----- ----- ----- 
		public class Loft_Measurements
		{
			float length = 137;
			float width = 183;

			public float wall_loft_north = 137;

			public float wall_loft_south = 137;
			public float wall_loft_south_doorEdge = 3.25f;
			public float wall_loft_south_doorGap = 30f;
			public float wall_loft_south_middle = 97.75f;
			public float wall_loft_south_railingEdge = 6;

			public float wall_loft_east = 183;
			public float wall_loft_east_left = 27.5f;
			public float wall_loft_east_railng = 118;
			public float wall_loft_east_right = 37.5f;

			public float wall_loft_west = 183;
			public float wall_loft_west_sidewall = 31.75f;
			public float wall_loft_west_railing = 151.75f;
		}
		public class UpperBedroomEntry_Measurements
		{
			float length = 54.75f;
			float width = 87f;

			public float wall_south_doorWall = 54.75f;

			public float wall_upperBedroom_east_doorWall = 87; // door wall

		}

		// ----- ----- ----- ----- ----- ----- ----- Upper Rm_Bed ----- ----- ----- ----- ----- ----- ----- 
		public class UpperBedroom_Measurements
		{
			float length = 75.75f;
			float width = 165f;

			public float wall_north = 130.5f - 54.75f;

			public float wall_south = 76f;
				public float wall_south_closetWall = 76f;

			public float wall_east = 165f;
				public float wall_upperBedroom_east_doorWall = 87f; // door wall
				public float wall_upperBedroom_east_windowWall = 78.25f; // window wall

			public float wall_west = 165f;
		}
		// ----- ----- ----- ----- ----- ----- ----- Upper Rm_Bed Closet ----- ----- ----- ----- ----- ----- ----- 
		public class UpperBedroomCloset_Measurements
		{
			float length = 76;
			float width = 23.75f;

			public float wall_north = 76;
			public float wall_north_right = 15;
			public float wall_north_left = 15;
			public float wall_north_gap = 46;

			public float wall_south = 76;

			public float wall_east = 23.75f;

			public float wall_west = 23.75f;
		}
		// ----- ----- ----- ----- ----- ----- ----- Deck Covered ----- ----- ----- ----- ----- ----- ----- 
		public class DeckCovered_Measurements
		{
			float length = 40f;
			float width	 = 158f;

			public float railing_west = 185f;

			public float railing_north = 68.5f;

			public float wall_door_east = 152.5f;
			public float wall_window_east = 27f;

			public float wall_south = 40f;
			public float wall_window_south = 28f;

		}

		// ----- ----- ----- ----- ----- ----- ----- Deck Uncovered ----- ----- ----- ----- ----- ----- ----- 
		public class DeckUncovered_Measurements
		{

			float length = 28f;
			float width = 185f;

			public float railing_west = 185f;

			public float railing_north = 68.5f;

			public float wall_door_east = 152.5f;
			public float wall_window_east = 27f;

			public float wall_window_south = 28f;
			public float wall_south = 40f;

		}

		// NOTES

		// EntryWay							— 40.25" × 43"				= 12.01		//sqft
		// EntryCloset					— 35" × 57"						= 13.85		//sqft
		// Hallway							— 36" × 229"					= 57.25		//sqft
		// Bathroom							— 63.75" × 71.25"			= 31.52		//sqft
		// Bedroom							— 157" × 148"					= 161.24	//sqft
		// BedroomCloset				— 78" × 71"					  = 38.42		//sqft
		// Storeroom						— 126" × 200"					= 175.00	//sqft
		// Kitchen							— 90.5" × 114"				= 71.63		//sqft
		// Laundry							— 89.5" × 31.25"			= 19.42		//sqft
		// Rm_Living						— 158.25" × 152"			= 166.96	//sqft
		// Rm_Dining						— 95.25" × 125.75"		= 83.23		//sqft
		// Loft									— 137" × 183"					= 174.04	//sqft
		// SecondBedroom				— 130.5" × 165"				= 149.53	//sqft
		// SecondBedroomCloset	— 76" × 23.75"				= 12.53		//sqft
		// TOTAL = 1,166.63 sq ft
		// LVP SQFT = 659.06

}

