using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class RoomIndex : MonoBehaviour
{
	// Walls - inches

	// Floor Level Y Positions
	public static float LevelOneHeight = 1.0f; // Height of the first floor in Unity units
	public static float LevelTwoHeight = 1.0f + 120f; // Height of the second floor in Unity units (10' x 12")

	public float LevelOneWallHeight = 93.5f; // Height of the first floor walls in Unity units

	// Floor Types
	public static List<string> FloorType = new List<string>() { "LVP", "Carpet", "Tile", "Rubber" };
	
	// Origin Point for Room Generation
	public static Vector3 Rm_SpawnPos_Origin = new Vector3(0, LevelOneHeight, 0); // Starting Reference (Furthest North-West Corner of the House)

	// Class ID Initializers
	private static int _floorID  = 1;
	private static int _vertexID = 1;
	private static int _wallID   = 1;

	public static List<FloorSection> All_FloorSectionList = new List<FloorSection>();
	public static List<WallSection>  All_WallSectionList	= new List<WallSection>();


	// ----- ----- ----- ----- ----- ----- ----- FloorSection Class ----- ----- ----- ----- ----- ----- ----- 

	public class FloorSection
	{

		public  int     ID;
		public  string  Name;
		public	string  Rm_Name;
		public  int     Level;
		public  float   Width;  // North - South 
		public  float   Length; // East - West
		public  float   Sqft;
		public  string  FloorType;
		public  bool	  IsActive;
		public  Vector3 SpawnPos;
		
		public  List<Rm_Vertex> VxList;

		// Always returns the real runtime class name (e.g. "Rm_Bath")
		public string ClassRef => GetType().Name;

		private const float SqftConversion = 144f; // 12x12 inches

		// -----  Constructor ----- 
		public FloorSection(string name, string rm_Name, int level, float width, float length, string floorType, bool isActive = true, Vector3 spawnPos = default(Vector3))
		{
			ID				= _floorID;
			_floorID	= _floorID + 1;

			Name      = name;
			Rm_Name   = rm_Name;
			Level     = level;
			SpawnPos  = spawnPos;
			Width     = width;
			Length    = length;
			Sqft      = length * width / SqftConversion;
			FloorType = floorType;
			IsActive  = isActive;

			All_FloorSectionList.Add(this);
			VxList = new List<Rm_Vertex>();
		}
	}
	


	// ----- ----- ----- ----- ----- ----- ----- Initializer ----- ----- ----- ----- ----- ----- -----

	public static FloorSection Rm_Living				 = new FloorSection("LivingRoom",					"Rm_Living",				 1, 151.75f,   152f, "LVP");
	public static FloorSection Rm_Dining				 = new FloorSection("DiningRoom",					"Rm_Dining",				 1, 125.75f, 95.25f, "LVP");
	public static FloorSection Rm_Hallway				 = new FloorSection("Hallway",						"Rm_Hallway",				 1,    229f,    36f, "LVP");
	public static FloorSection Rm_Bed						 = new FloorSection("Bedroom",						"Rm_Bed",						 1,    157f,   148f, "LVP");
	public static FloorSection Rm_Kitchen				 = new FloorSection("Kitchen",						"Rm_Kitchen",				 1,    114f,  90.5f, "LVP");
	
	public static FloorSection Rm_Laundry				 = new FloorSection("Laundry",					  "Rm_Laundry",				 1,  35.75f,  90.5f, "LVP");
	public static FloorSection Rm_EntryWay			 = new FloorSection("EntryWay",						"Rm_EntryWay",			 1,     43f, 40.25f, "LVP");
	public static FloorSection Rm_EntryCloset		 = new FloorSection("EntryCloset",				"Rm_EntryCloset",		 1,  56.75f,    35f, "LVP");
	public static FloorSection Rm_Bath					 = new FloorSection("Bathroom",						"Rm_Bath",					 1,  71.25f,    87f, "LVP");
	public static FloorSection Rm_BedCloset			 = new FloorSection("BedroomCloset",			"Rm_BedCloset",			 1,     71f,    78f, "LVP");
	
	public static FloorSection Rm_StoreEntry		 = new FloorSection("StoreroomEntry",			"Rm_StoreEntry",		 1,   65.5f,   103f, "Carpet");
	public static FloorSection Rm_Store					 = new FloorSection("Storeroom",					"Rm_Store",					 1,    120f, 127.5f, "Carpet");
	public static FloorSection Rm_DeckCovered		 = new FloorSection("DeckCovered",				"Rm_DeckCovered",		 1,    158f,    40f, "Rubber");
	public static FloorSection Rm_DeckUncovered	 = new FloorSection("DeckUncovered",			"Rm_DeckUncovered",  1,    185f,    28f, "Rubber");
	
	// 2nd Floor
	public static FloorSection Rm_Loft					 = new FloorSection("Loft",								"Rm_Loft",					 2,    183f,   137f, "Carpet");
	public static FloorSection Rm_UpperBed			 = new FloorSection("UpperBedroom",				"Rm_UpperBed",			 2,    165f,    76f, "Carpet");
	public static FloorSection Rm_UpperBedEntry	 = new FloorSection("UpperBedroomEntry",	"Rm_UpperBedEntry",	 2,     87f, 54.75f, "Carpet");
	public static FloorSection Rm_UpperBedCloset = new FloorSection("UpperBedroomCloset", "Rm_UpperBedCloset", 2,  23.75f,    76f, "Carpet");



	public void Start()
	{
		AssignVertexLists();

		Assign_Wall_SpawnPositions();

		ConsoleLogVertexList(All_FloorSectionList);
	}



	// ----- ----- ----- ----- ----- ----- ----- Rm_Vertex Class ----- ----- ----- ----- ----- ----- ----- 
	public class Rm_Vertex
	{
		private int			ID;
		public	string	Name;
		public	int			Level;	
		public	Vector3 Position;
		public	int			Order;  // Order of the vertex in the room (0 = BottomLeft, 1 = BottomRight, 2 = TopRight, 3 = TopLeft)
		public	bool		IsActive;

		// -----  Constructor ----- 
		public Rm_Vertex(string name, int level, Vector3 position, int order = 0, bool isActive = true)
		{
			ID				 = _vertexID;
			_vertexID  = _vertexID + 1;

			Name			 = name;
			Level			 = level;
			Position	 = position;
			Order			 = order;
			IsActive	 = isActive;
		}
	}

	public void Generate_VertexList(FloorSection floor)
	{
		Vector3 BottomLeft	= new Vector3(floor.SpawnPos.x,										 floor.SpawnPos.y,	floor.SpawnPos.z);
		Vector3 BottomRight = new Vector3(floor.SpawnPos.x + floor.Width,			 floor.SpawnPos.y,	floor.SpawnPos.z);
		Vector3 TopRight		= new Vector3(floor.SpawnPos.x + floor.Width,			 floor.SpawnPos.y,  floor.SpawnPos.z + floor.Length);
		Vector3 TopLeft			= new Vector3(floor.SpawnPos.x,										 floor.SpawnPos.y,	floor.SpawnPos.z + floor.Length);

		string Vx_Name = "Vx_" + floor.Rm_Name.Substring(3);

		// Ex: Rm_Vertex Vx_TopLeft = new Rm_Vertex("Vx_Living_TopLeft", 1, Rm_Living_TopLeft);
		Rm_Vertex Vx_BottomLeft			= new Rm_Vertex(Vx_Name + "_" + nameof(BottomLeft),	 floor.Level, BottomLeft,	 0);
		Rm_Vertex Vx_BottomRight		= new Rm_Vertex(Vx_Name + "_" + nameof(BottomRight), floor.Level, BottomRight, 1);
		Rm_Vertex Vx_TopRight				= new Rm_Vertex(Vx_Name + "_" + nameof(TopRight),		 floor.Level, TopRight,		 2);
		Rm_Vertex Vx_TopLeft				= new Rm_Vertex(Vx_Name + "_" + nameof(TopLeft),		 floor.Level, TopLeft,		 3);

		List<Rm_Vertex> VertexList = new List<Rm_Vertex> {
			Vx_BottomLeft,
			Vx_BottomRight,
			Vx_TopRight,
			Vx_TopLeft
		};

		// Update FloorSection floor
		floor.VxList = VertexList;

	}



		// Helper to safely get a vertex by Order
	Rm_Vertex GetVertex(FloorSection room, int order)
	{
		if (room == null || room.VxList == null)
		{
			Debug.LogError($"Room or VxList is null when looking for Order {order}");
			return null;
		}

		Rm_Vertex vertex = room.VxList.Find(v => v.Order == order);

		if (vertex == null)
		{
			Debug.LogError($"Could not find vertex with Order {order} on room '{room.Name}'");
		}

		return vertex;
	}




	public void AssignVertexLists()
	{
		foreach (FloorSection floor in All_FloorSectionList)		{
			Generate_VertexList(floor);
		}
		
		Assign_Floor_SpawnPositions();

	}

	// Assign Spawn Positions for each room based on the vertex positions
	public void Assign_Floor_SpawnPositions()
	{

		// ----- Origin -----
		Rm_Living.SpawnPos = Rm_SpawnPos_Origin;
		Generate_VertexList(Rm_Living);

		// ----- First floor chain
		var livingTopLeft = GetVertex(Rm_Living, 3);
		if (livingTopLeft != null)
		{
			Rm_Dining.SpawnPos = livingTopLeft.Position;
		}
		Generate_VertexList(Rm_Dining);

		//----- Hallway (offset on Z)
		var livingTopRight = GetVertex(Rm_Living, 2);
		if (livingTopRight != null)
		{
			Rm_Hallway.SpawnPos = new Vector3(livingTopRight.Position.x, livingTopRight.Position.y, livingTopRight.Position.z - Rm_Hallway.Length);
		}
		Generate_VertexList(Rm_Hallway);

		//----- Bed (offset on Z)	
		var livingBottomRight = GetVertex(Rm_Hallway, 0);
		if (livingBottomRight != null)
		{
			Vector3 org = livingBottomRight.Position;
			Rm_Bed.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_Bed.Length);
		}
		Generate_VertexList(Rm_Bed);

		//----- Kitchen (offset on Z)
		var diningTopRight = GetVertex(Rm_Dining, 2);
		if (diningTopRight != null)
		{
			Vector3 org = diningTopRight.Position;
			Rm_Kitchen.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_Kitchen.Length);
		}
		Generate_VertexList(Rm_Kitchen);

		//----- Laundry (offset on Z)
		var kitchenTopRight = GetVertex(Rm_Kitchen, 2);
		if (kitchenTopRight != null)
		{
			Vector3 org = kitchenTopRight.Position;
			Rm_Laundry.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_Laundry.Length);
		}
		Generate_VertexList(Rm_Laundry);

		//----- EntryWay (offset on Z)
		var laundryBottomRight = GetVertex(Rm_Laundry, 1);
		if (laundryBottomRight != null)
		{
			Vector3 org = laundryBottomRight.Position;
			Rm_EntryWay.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z);
		}
		Generate_VertexList(Rm_EntryWay);

		//----- EntryCloset (offset on Z)
		var entryWayTopRight = GetVertex(Rm_EntryWay, 2);
		if (entryWayTopRight != null)
		{
			Vector3 org = entryWayTopRight.Position;
			Rm_EntryCloset.SpawnPos = new Vector3(
			org.x, 
			org.y, 
			org.z - Rm_EntryCloset.Length);
		}
		Generate_VertexList(Rm_EntryCloset);

		//----- Bath (offset on Z)
		var hallwayBottomRight = GetVertex(Rm_Bed, 2);
		if (hallwayBottomRight != null)		{
			Vector3 org = hallwayBottomRight.Position;
			Rm_Bath.SpawnPos = 
			new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_Bath.Length);
		}
		Generate_VertexList(Rm_Bath);

		//----- BedCloset (offset on Z)
		var bathBottomRight = GetVertex(Rm_Bath, 0);
		if (bathBottomRight != null)
		{
			Vector3 org = bathBottomRight.Position;
			Rm_BedCloset.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_BedCloset.Length);
		}
		Generate_VertexList(Rm_BedCloset);

		//----- StoreEntry (offset on Z)
		var bedClosetBottomRight = GetVertex(Rm_BedCloset, 0);
		if (bedClosetBottomRight != null)
		{
			Vector3 org = bedClosetBottomRight.Position;
			Rm_StoreEntry.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_StoreEntry.Length);
		}
		Generate_VertexList(Rm_StoreEntry);

		//----- Store (offset on Z)
		var storeEntryBottomLeft = GetVertex(Rm_StoreEntry, 0);
		if (storeEntryBottomLeft != null)
		{
			Vector3 org = storeEntryBottomLeft.Position;
			Rm_Store.SpawnPos = new Vector3(
				org.x - Rm_Store.Width, 
				org.y, 
				org.z);
		}
		Generate_VertexList(Rm_Store);

		//----- DeckCovered (offset on Z)
		var livingBottomLeft = GetVertex(Rm_Living, 0);
		if (livingBottomLeft != null)
		{
			Vector3 org = livingBottomLeft.Position;
			Rm_DeckCovered.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_DeckCovered.Length);
		}
		Generate_VertexList(Rm_DeckCovered);

		//----- DeckUncovered (offset on Z)
		var deckCoveredBottomLeft = GetVertex(Rm_DeckCovered, 0);
		if (deckCoveredBottomLeft != null)
		{
			Vector3 org = deckCoveredBottomLeft.Position;
			Rm_DeckUncovered.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_DeckUncovered.Length);
		}
		Generate_VertexList(Rm_DeckUncovered);

		// ----- Second floor -----

		//----- Loft (offset on Z)
		var diningTopLeft = GetVertex(Rm_Dining, 3);
		if (diningTopLeft != null)
		{
			Vector3 org = diningTopLeft.Position;
			Rm_Loft.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_Loft.Length);
		}
		Generate_VertexList(Rm_Loft);

		//----- UpperBedEntry (offset on Z)
		var loftTopRight = GetVertex(Rm_Loft, 2);
		if (loftTopRight != null)		{
			Vector3 org = loftTopRight.Position;
			Rm_UpperBedEntry.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_UpperBedEntry.Length);
		}
		Generate_VertexList(Rm_UpperBedEntry);

		//----- UpperBed (offset on Z)
		var upperBedEntryBottomLeft = GetVertex(Rm_UpperBedEntry, 0);
		if (upperBedEntryBottomLeft != null)
		{
			Vector3 org = upperBedEntryBottomLeft.Position;
			Rm_UpperBed.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z - Rm_UpperBed.Length);
		}
		Generate_VertexList(Rm_UpperBed);

		//----- UpperBedCloset (offset on Z)
		var upperBedBottomRight = GetVertex(Rm_UpperBed, 1);
		if (upperBedBottomRight != null)
		{
			Vector3 org = upperBedBottomRight.Position;
			Rm_UpperBedCloset.SpawnPos = new Vector3(
				org.x, 
				org.y, 
				org.z);
		}
		Generate_VertexList(Rm_UpperBedCloset);

	}

	#region OriginalSpawnPositionScript


	//public void Assign_SpawnPositions(){ 
	//	Rm_Living.SpawnPos		= Rm_SpawnPos_Origin;

	//	Rm_Dining.SpawnPos		= Rm_Living.VxList.Find(v => v.Order == 3).Position;

	//	Rm_Hallway.SpawnPos		= Rm_Living.VxList.Find(v => v.Order == 2).Position;

	//	Rm_Bed.SpawnPos				= Rm_Living.VxList.Find(v => v.Order == 1).Position;

	//	Vector3 org_Kitchen = Rm_Dining.VxList.Find(v => v.Order == 2).Position;
	//	Rm_Kitchen.SpawnPos	= new Vector3(org_Kitchen.x, org_Kitchen.y, org_Kitchen.z - Rm_Kitchen.Length);

	//	Vector3 org_Laundry = Rm_Kitchen.VxList				.Find(v => v.Order == 2).Position;
	//	Rm_Laundry.SpawnPos	= new Vector3(org_Laundry.x, org_Laundry.y, org_Laundry.z - Rm_Laundry.Length);

	//	Vector3 org_EntryWay = Rm_Laundry.VxList				.Find(v => v.Order == 1).Position;
	//	Rm_EntryWay.SpawnPos = new Vector3(org_EntryWay.x, org_EntryWay.y, org_EntryWay.z - Rm_EntryWay.Length);

	//	Vector3 org_EntryCloset = Rm_EntryWay.VxList			.Find(v => v.Order == 2).Position;
	//	Rm_EntryCloset.SpawnPos = new Vector3(org_EntryCloset.x, org_EntryCloset.y, org_EntryCloset.z - Rm_EntryCloset.Length);

	//	Vector3 org_Bath = Rm_Hallway.VxList				.Find(v => v.Order == 1).Position;
	//	Rm_Bath.SpawnPos = new Vector3(org_Bath.x, org_Bath.y, org_Bath.z - Rm_Bath.Length);

	//	Vector3 org_BedCloset = Rm_Bath.VxList					.Find(v => v.Order == 1).Position;
	//	Rm_BedCloset.SpawnPos = new Vector3(org_BedCloset.x, org_BedCloset.y, org_BedCloset.z - Rm_BedCloset.Length);

	//	Vector3 org_StoreEntry = Rm_BedCloset.VxList			.Find(v => v.Order == 1).Position;
	//	Rm_StoreEntry.SpawnPos = new Vector3(org_StoreEntry.x, org_StoreEntry.y, org_StoreEntry.z - Rm_StoreEntry.Length);

	//	Vector3 org_Store = Rm_StoreEntry.VxList		.Find(v => v.Order == 0).Position;
	//	Rm_Store.SpawnPos = new Vector3(org_Store.x, org_Store.y, org_Store.z - Rm_Store.Length);

	//	Vector3 org_DeckCovered = Rm_Living.VxList				.Find(v => v.Order == 0).Position;
	//	Rm_DeckCovered.SpawnPos = new Vector3(org_DeckCovered.x, org_DeckCovered.y, org_DeckCovered.z - Rm_DeckCovered.Length);

	//	Vector3 org_DeckUncovered = Rm_DeckCovered.VxList		.Find(v => v.Order == 0).Position;
	//	Rm_DeckUncovered.SpawnPos = new Vector3(org_DeckUncovered.x, org_DeckUncovered.y, org_DeckUncovered.z - Rm_DeckUncovered.Length);

	//	Vector3 org_Loft = Rm_Dining.VxList				.Find(v => v.Order == 3).Position;
	//	Rm_Loft.SpawnPos = new Vector3(org_Loft.x, org_Loft.y, org_Loft.z - Rm_Loft.Length);

	//	Vector3 org_UpperBedEntry = Rm_Loft.VxList					.Find(v => v.Order == 2).Position;
	//	Rm_UpperBedEntry.SpawnPos = new Vector3(org_UpperBedEntry.x, org_UpperBedEntry.y, org_UpperBedEntry.z - Rm_UpperBedEntry.Length);

	//	Vector3 org_UpperBed = Rm_UpperBedEntry.VxList	.Find(v => v.Order == 0).Position;
	//	Rm_UpperBed.SpawnPos = new Vector3(org_UpperBed.x, org_UpperBed.y, org_UpperBed.z - Rm_UpperBed.Length);

	//	Vector3 org_UpperBedCloset = Rm_UpperBed.VxList			.Find(v => v.Order == 1).Position;
	//	Rm_UpperBedCloset.SpawnPos = new Vector3(org_UpperBedCloset.x, org_UpperBedCloset.y, org_UpperBedCloset.z - Rm_UpperBedCloset.Length);
	//}

	#endregion

	//Spawn Order
	//  Rm_Living
	//  Rm_Dining
	//  Rm_Hallway
	//  Rm_Bed
	//  Rm_Kitchen
	//  Rm_Laundry
	//  Rm_EntryWay
	//  Rm_EntryCloset
	//  Rm_Bath
	//  Rm_BedCloset
	//  Rm_StoreEntry
	//  Rm_Store
	//  Rm_DeckCovered
	//  Rm_DeckUncovered
	//  Rm_Loft
	//  Rm_UpperBedEntry
	//  Rm_UpperBed
	//  Rm_UpperBedCloset






	public void ConsoleLogVertexList(List<FloorSection> topList)
	{
		string consoleLog = "Room Vertices: \n";

		if (topList == null || topList.Count == 0){ 
			Debug.Log("No Floors in List.");
			return;
		}
		
		consoleLog += $"Room Vertices in {topList}: \n";
		foreach (FloorSection floor in topList) 
		{
			consoleLog += $"--{floor.Name} \n";
			foreach (Rm_Vertex vx in floor.VxList) 
			{
				consoleLog +=
					$"----{vx.Name} \n" +
					$"------ {vx.Position.x}, {vx.Position.y}, {vx.Position.z}\n";
			}
		}
		Debug.Log(consoleLog);
	}

	

	// ----- ----- ----- ----- ----- ----- ----- WallSection Class ----- ----- ----- ----- ----- ----- ----- 

	public enum WL_Direction
	{
		NS = 1, // North-South direction (X-axis)
		EW = 2  // East-West direction	 (Z-axis)
	}

	public static float WL_Thick = 5; // Wall thickness in inches

	public class WallSection
	{

		public int					ID;
		public string				Name;
		public int					Level;
		public WL_Direction Direction;
		public float				Width;
		public float				Length;
		public bool					IsActive;
		public Vector3			SpawnPos;



		// -----  Constructor ----- 
		public WallSection(string name, int level, WL_Direction direction, float width, float length, bool isActive = true, Vector3 spawnPos = default(Vector3))
		{
			ID = _wallID;
			_wallID = _wallID + 1;

			Name			= name;
			Level			= level;
			Direction = direction;
			Width			= width;
			Length		= length;
		}
	}


	public void Assign_Wall_SpawnPositions()
	{
		// Example: Assigning spawn positions for wall sections based on room vertices
		// This is a placeholder; actual logic will depend on the specific layout and requirements

		// Wall Naming Convention - WL_ + Wall Section Name referenced for Vertex Spawn Position + Adjacent Shared Wall
		// [WL]_[FloorSectionSpawnReference]_[OppositeFloorSection]


		// Kitchen Wall (North-South) wall - 98 Length
		Vector3 WL_SpawnPos_Kitchen_Hallway = (Rm_Kitchen.VxList.Find(v => v.Order == 0).Position + new Vector3(-14, 0, -WL_Thick));
		WallSection WL_Kitchen_HallWay = new WallSection(nameof(WL_Kitchen_HallWay), 1, WL_Direction.NS, 98, WL_Thick);
		WL_Kitchen_HallWay.SpawnPos = WL_SpawnPos_Kitchen_Hallway;

		// Kitchen Wall Exterior (North-South) wall - 98 Length
		Vector3 WL_SpawnPos_Kitchen_Exterior_East = (Rm_Kitchen.VxList.Find(v => v.Order == 3).Position + new Vector3(0, 0, 0));
		WallSection WL_Kitchen_Exterior_East = new WallSection(nameof(WL_Kitchen_Exterior_East), 1, WL_Direction.NS, 114, WL_Thick);
		WL_Kitchen_Exterior_East.SpawnPos = WL_SpawnPos_Kitchen_Exterior_East;



		// Laundry Room (North-South) wall - 40.75 Length
		Vector3 WL_SpawnPos_Laundry_Hallway = (Rm_Laundry.VxList.Find(v => v.Order == 0).Position + new Vector3(0, 0, -5));
		WallSection WL_Laundry_HallWay = new WallSection(nameof(WL_Laundry_HallWay), 1, WL_Direction.NS, 40.75f, WL_Thick);
		WL_Laundry_HallWay.SpawnPos = WL_SpawnPos_Laundry_Hallway;

		// Laundry Room (East-West) wall - 90.5 Length
		Vector3 WL_SpawnPos_Laundry_EntryWay = (Rm_Laundry.VxList.Find(v => v.Order == 1).Position + new Vector3(0, 0, 0));
		WallSection WL_Laundry_EntryWay = new WallSection(nameof(WL_Laundry_EntryWay), 1, WL_Direction.EW, WL_Thick, 90.5f, true);
		WL_Laundry_EntryWay.SpawnPos = WL_SpawnPos_Laundry_EntryWay;

		// Laundry Room Exterior East (East-West) wall - 35.75 Length
		Vector3 WL_SpawnPos_Laundry_Exterior_East = (Rm_Laundry.VxList.Find(v => v.Order == 3).Position + new Vector3(0, 0, 0));
		WallSection WL_Laundry_Exterior_East = new WallSection(nameof(WL_Laundry_Exterior_East), 1, WL_Direction.EW, 35.75f, WL_Thick, true);
		WL_Laundry_Exterior_East.SpawnPos = WL_SpawnPos_Laundry_Exterior_East;





		// Entryway Closet (East-West) wall - 5 Length
		Vector3 WL_SpawnPos_EntryCloset_Entryway = (Rm_EntryCloset.VxList.Find(v => v.Order == 3).Position + new Vector3(WL_Thick, 0, -WL_Thick));
		WallSection WL_EntryCloset_Entry = new WallSection(nameof(WL_EntryCloset_Entry), 1, WL_Direction.EW, WL_Thick, 5.0f, true);
		WL_EntryCloset_Entry.SpawnPos = WL_SpawnPos_EntryCloset_Entryway;

		// Entryway (North-South) wall - 40.25 Length



		// Living Room (East-West) wall - 151.75 Length
		Vector3 WL_SpawnPos_Living_Exterior = (Rm_Living.VxList.Find(v => v.Order == 0).Position + new Vector3(-WL_Thick, 0, 0));
		WallSection WL_Living_Exterior = new WallSection(nameof(WL_Living_Exterior), 1, WL_Direction.EW, WL_Thick, 151.75f);
		WL_Living_Exterior.SpawnPos = WL_SpawnPos_Living_Exterior;

		// Living Room (North-South) wall - 152 Length
		Vector3 WL_SpawnPos_Living_DeckCovered = (Rm_Living.VxList.Find(v => v.Order == 0).Position + new Vector3(0, 0, -WL_Thick));
		WallSection WL_Living_DeckCovered = new WallSection(nameof(WL_Living_DeckCovered), 1, WL_Direction.NS, 152.0f, WL_Thick);
		WL_Living_DeckCovered.SpawnPos = WL_SpawnPos_Living_DeckCovered;



		// Dinging Room (East-West) wall - 95.25 Length
		Vector3 WL_SpawnPos_Dining_Exterior_North = (Rm_Dining.VxList.Find(v => v.Order == 0).Position + new Vector3(-WL_Thick, 0, 0));
		WallSection WL_Dining_Exterior_North = new WallSection(nameof(WL_Dining_Exterior_North), 1, WL_Direction.EW, WL_Thick, 95.25f);
		WL_Dining_Exterior_North.SpawnPos = WL_SpawnPos_Dining_Exterior_North;

		// Dining Room (North-South) wall - 125.75 Length
		Vector3 WL_SpawnPos_Dining_Exterior_East = (Rm_Dining.VxList.Find(v => v.Order == 3).Position + new Vector3(0, 0, 0));
		WallSection WL_Dining_Exterior_East = new WallSection(nameof(WL_Dining_Exterior_East), 1, WL_Direction.NS, 125.75f, WL_Thick);
		WL_Dining_Exterior_East.SpawnPos = WL_SpawnPos_Dining_Exterior_East;




		// Bedroom (East-West) wall - 148 Length
		Vector3 WL_SpawnPos_Bed_Living = (Rm_Bed.VxList.Find(v => v.Order == 0).Position + new Vector3(-WL_Thick, 0, 0));
		WallSection WL_Bed_Living = new WallSection(nameof(WL_Bed_Living), 1, WL_Direction.EW, WL_Thick, 148f);
		WL_Bed_Living.SpawnPos = WL_SpawnPos_Bed_Living;

		// Bedroom (North-South) wall - 157 Length
		Vector3 WL_SpawnPos_Bed_HallWay = (Rm_Bed.VxList.Find(v => v.Order == 3).Position + new Vector3(0, 0, 0));
		WallSection WL_Bed_HallWay = new WallSection(nameof(WL_Bed_HallWay), 1, WL_Direction.NS, 157f, WL_Thick);
		WL_Bed_HallWay.SpawnPos = WL_SpawnPos_Bed_HallWay;




		// Bathroom (East-West) wall - 87 Length
		Vector3 WL_SpawnPos_Bath_Exterior_South = (Rm_Bath.VxList.Find(v => v.Order == 1).Position + new Vector3(0, 0, 0));
		WallSection WL_Bath_Exterior_South = new WallSection(nameof(WL_Bath_Exterior_South), 1, WL_Direction.EW, WL_Thick, 87);
		WL_Bath_Exterior_South.SpawnPos = WL_SpawnPos_Bath_Exterior_South;

		// Bathroom (North-South) wall - 71.25 Length
		Vector3 WL_SpawnPos_Bath_Hallways = (Rm_Bath.VxList.Find(v => v.Order == 3).Position + new Vector3(0, 0, 0));
		WallSection WL_Bath_Hallways = new WallSection(nameof(WL_Bath_Hallways), 1, WL_Direction.NS, 71.25f, WL_Thick);
		WL_Bath_Hallways.SpawnPos = WL_SpawnPos_Bath_Hallways;



		// Hallway (East-West) wall - 36 Length
		Vector3 WL_SpawnPos_Hallway_Exterior_South = (Rm_Hallway.VxList.Find(v => v.Order == 10).Position + new Vector3(0, 0, 0));
		WallSection WL_Hallway_Exterior_South = new WallSection(nameof(WL_Hallway_Exterior_South), 1, WL_Direction.EW, WL_Thick, 36f);
		WL_Hallway_Exterior_South.SpawnPos = WL_SpawnPos_Hallway_Exterior_South;







		All_WallSectionList.Add(WL_Kitchen_HallWay);
		All_WallSectionList.Add(WL_Kitchen_Exterior_East);

		All_WallSectionList.Add(WL_Laundry_HallWay);
		All_WallSectionList.Add(WL_Laundry_EntryWay);
		All_WallSectionList.Add(WL_Laundry_Exterior_East);

		All_WallSectionList.Add(WL_Entryway_EntryCloset);

		All_WallSectionList.Add(WL_Living_Exterior);
		All_WallSectionList.Add(WL_Living_DeckCovered);

		All_WallSectionList.Add(WL_Dining_Exterior_North);
		All_WallSectionList.Add(WL_Dining_Exterior_East);


		All_WallSectionList.Add(WL_Bed_Living);
		All_WallSectionList.Add(WL_Bed_HallWay);

		All_WallSectionList.Add(WL_Bath_Exterior_South);
		All_WallSectionList.Add(WL_Bath_Hallways);

		All_WallSectionList.Add(WL_Hallway_Exterior_South);



	}







	#region Original Vertex Calculation Code

	//public void Calculate_Vertices()
	//{

	//	// 1 Rm_Living_Vertices;
	//	Vector3 Rm_SpawnPos_Living = new Vector3(0, LevelOneHeight, 0); // Starting Reference (Furthest North-West Corner of the House)

	//	Vector3 Rm_Living_TopLeft     = new Vector3(Rm_SpawnPos_Living.x,										Rm_SpawnPos_Living.y, Rm_SpawnPos_Living.z + Rm_Living.Length);
	//	Vector3 Rm_Living_TopRight    = new Vector3(Rm_SpawnPos_Living.x + Rm_Living.Width, Rm_SpawnPos_Living.y, Rm_SpawnPos_Living.z + Rm_Living.Length);
	//	Vector3 Rm_Living_BottomLeft  = new Vector3(Rm_SpawnPos_Living.x,										Rm_SpawnPos_Living.y, Rm_SpawnPos_Living.z);
	//	Vector3 Rm_Living_BottomRight = new Vector3(Rm_SpawnPos_Living.x + Rm_Living.Width, Rm_SpawnPos_Living.y, Rm_SpawnPos_Living.z);

	//	//List< (string name, Vector3 Pos)> Rm_CornerList_Living = new List<(string name, Vector3 Pos)> { (Rm_Living_TopLeft, Rm_Living_TopRight, Rm_Living_BottomLeft, Rm_Living_BottomRight };
	//	List<Vector3> Rm_CornerList_Living = new List<Vector3> { Rm_Living_TopLeft, Rm_Living_TopRight, Rm_Living_BottomLeft, Rm_Living_BottomRight};

	//	List<Rm_Vertex> Rm_VertexList_Living = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Living_TopLeft",     1, Rm_Living_TopLeft),
	//		GenerateVertex("Vx_Living_TopRight",    1, Rm_Living_TopRight),
	//		GenerateVertex("Vx_Living_BottomLeft",  1, Rm_Living_BottomLeft),
	//		GenerateVertex("Vx_Living_BottomRight", 1, Rm_Living_BottomRight)
	//	};


	//	// 2 Rm_Dining_Vertices()
	//	Vector3 Rm_SpawnPos_Dining = Rm_Living_TopLeft;

	//	Vector3 Rm_Dining_TopLeft			= new Vector3(Rm_SpawnPos_Dining.x,										Rm_SpawnPos_Dining.y, Rm_SpawnPos_Dining.z + Rm_Dining.Length);
	//	Vector3 Rm_Dining_TopRight		= new Vector3(Rm_SpawnPos_Dining.x + Rm_Dining.Width, Rm_SpawnPos_Dining.y, Rm_SpawnPos_Dining.z + Rm_Dining.Length);
	//	Vector3 Rm_Dining_BottomLeft	= new Vector3(Rm_SpawnPos_Dining.x,										Rm_SpawnPos_Dining.y, Rm_SpawnPos_Dining.z);
	//	Vector3 Rm_Dining_BottomRight	= new Vector3(Rm_SpawnPos_Dining.x + Rm_Dining.Width, Rm_SpawnPos_Dining.y, Rm_SpawnPos_Dining.z);

	//	List<Vector3> Rm_CornerList_Dining = new List<Vector3> { Rm_Dining_TopLeft, Rm_Dining_TopRight, Rm_Dining_BottomLeft, Rm_Dining_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Dining = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Dining_TopLeft",			1, Rm_Dining_TopLeft),
	//		GenerateVertex("Vx_Dining_TopRight",		1, Rm_Dining_TopRight),
	//		GenerateVertex("Vx_Dining_BottomLeft",	1, Rm_Dining_BottomLeft),
	//		GenerateVertex("Vx_Dining_BottomRight", 1, Rm_Dining_BottomRight)
	//	};


	//	// 3 Rm_Hallway_Vertices()
	//	Vector3 Rm_SpawnPos_Hallway		 = new Vector3(Rm_Living_TopRight.x, LevelOneHeight, Rm_Living_TopRight.z - Rm_Hallway.Length);

	//	Vector3 Rm_Hallway_TopLeft		 = new Vector3(Rm_SpawnPos_Hallway.x,										 Rm_SpawnPos_Hallway.y, Rm_SpawnPos_Hallway.z + Rm_Hallway.Length);
	//	Vector3 Rm_Hallway_TopRight		 = new Vector3(Rm_SpawnPos_Hallway.x + Rm_Hallway.Width, Rm_SpawnPos_Hallway.y, Rm_SpawnPos_Hallway.z + Rm_Hallway.Length);
	//	Vector3 Rm_Hallway_BottomLeft	 = new Vector3(Rm_SpawnPos_Hallway.x,										 Rm_SpawnPos_Hallway.y, Rm_SpawnPos_Hallway.z);
	//	Vector3 Rm_Hallway_BottomRight = new Vector3(Rm_SpawnPos_Hallway.x + Rm_Hallway.Width, Rm_SpawnPos_Hallway.y, Rm_SpawnPos_Hallway.z);

	//	List<Vector3> Rm_CornerList_Hallway = new List<Vector3> { Rm_Hallway_TopLeft, Rm_Hallway_TopRight, Rm_Hallway_BottomLeft, Rm_Hallway_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Hallway = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Hallway_TopLeft",		 1, Rm_Hallway_TopLeft),
	//		GenerateVertex("Vx_Hallway_TopRight",		 1, Rm_Hallway_TopRight),
	//		GenerateVertex("Vx_Hallway_BottomLeft",	 1, Rm_Hallway_BottomLeft),
	//		GenerateVertex("Vx_Hallway_BottomRight", 1, Rm_Hallway_BottomRight)
	//	};


	//	// 4 Rm_Bed_Vertices()
	//	Vector3 Rm_SpawnPos_Bed				 = new Vector3(Rm_Living_BottomRight.x, LevelOneHeight, Rm_Living_BottomRight.z - Rm_Hallway.Length);

	//	Vector3 Rm_Bed_TopLeft		 = new Vector3(Rm_SpawnPos_Bed.x,								 Rm_SpawnPos_Bed.y, Rm_SpawnPos_Bed.z + Rm_Bed.Length);
	//	Vector3 Rm_Bed_TopRight		 = new Vector3(Rm_SpawnPos_Bed.x + Rm_Bed.Width, Rm_SpawnPos_Bed.y, Rm_SpawnPos_Bed.z + Rm_Bed.Length);
	//	Vector3 Rm_Bed_BottomLeft	 = new Vector3(Rm_SpawnPos_Bed.x,								 Rm_SpawnPos_Bed.y, Rm_SpawnPos_Bed.z);
	//	Vector3 Rm_Bed_BottomRight = new Vector3(Rm_SpawnPos_Bed.x + Rm_Bed.Width, Rm_SpawnPos_Bed.y, Rm_SpawnPos_Bed.z);

	//	List<Vector3> Rm_CornerList_Bed = new List<Vector3> { Rm_Bed_TopLeft, Rm_Bed_TopRight, Rm_Bed_BottomLeft, Rm_Bed_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Bed = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Bed_TopLeft",		 1, Rm_Bed_TopLeft),
	//		GenerateVertex("Vx_Bed_TopRight",		 1, Rm_Bed_TopRight),
	//		GenerateVertex("Vx_Bed_BottomLeft",	 1, Rm_Bed_BottomLeft),
	//		GenerateVertex("Vx_Bed_BottomRight", 1, Rm_Bed_BottomRight)
	//	};


	//	// 5 Rm_Kitchen_Vertices()
	//	Vector3 Rm_SpawnPos_Kitchen = new Vector3(Rm_Dining_TopRight.x, LevelOneHeight, Rm_Dining_TopRight.z - Rm_Kitchen.Length);

	//	Vector3 Rm_Kitchen_TopLeft		 = new Vector3(Rm_SpawnPos_Kitchen.x,										 Rm_SpawnPos_Kitchen.y, Rm_SpawnPos_Kitchen.z + Rm_Kitchen.Length);
	//	Vector3 Rm_Kitchen_TopRight		 = new Vector3(Rm_SpawnPos_Kitchen.x + Rm_Kitchen.Width, Rm_SpawnPos_Kitchen.y, Rm_SpawnPos_Kitchen.z + Rm_Kitchen.Length);
	//	Vector3 Rm_Kitchen_BottomLeft	 = new Vector3(Rm_SpawnPos_Kitchen.x,										 Rm_SpawnPos_Kitchen.y, Rm_SpawnPos_Kitchen.z);
	//	Vector3 Rm_Kitchen_BottomRight = new Vector3(Rm_SpawnPos_Kitchen.x + Rm_Kitchen.Width, Rm_SpawnPos_Kitchen.y, Rm_SpawnPos_Kitchen.z);

	//	List<Vector3> Rm_CornerList_Kitchen = new List<Vector3> { Rm_Kitchen_TopLeft, Rm_Kitchen_TopRight, Rm_Kitchen_BottomLeft, Rm_Kitchen_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Kitchen = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Kitchen_TopLeft",		 1, Rm_Kitchen_TopLeft),
	//		GenerateVertex("Vx_Kitchen_TopRight",		 1, Rm_Kitchen_TopRight),
	//		GenerateVertex("Vx_Kitchen_BottomLeft",	 1, Rm_Kitchen_BottomLeft),
	//		GenerateVertex("Vx_Kitchen_BottomRight", 1, Rm_Kitchen_BottomRight)
	//	};


	//	// 6 Rm_Laundry_Vertices()
	//	Vector3 Rm_SpawnPos_Laundry		 = new Vector3(Rm_Kitchen_TopRight.x, LevelOneHeight, Rm_Kitchen_TopRight.z - Rm_Laundry.Length);

	//	Vector3 Rm_Laundry_TopLeft		 = new Vector3(Rm_SpawnPos_Laundry.x,										 Rm_SpawnPos_Laundry.y, Rm_SpawnPos_Laundry.z + Rm_Laundry.Length);
	//	Vector3 Rm_Laundry_TopRight		 = new Vector3(Rm_SpawnPos_Laundry.x + Rm_Laundry.Width, Rm_SpawnPos_Laundry.y, Rm_SpawnPos_Laundry.z + Rm_Laundry.Length);
	//	Vector3 Rm_Laundry_BottomLeft	 = new Vector3(Rm_SpawnPos_Laundry.x,										 Rm_SpawnPos_Laundry.y, Rm_SpawnPos_Laundry.z);
	//	Vector3 Rm_Laundry_BottomRight = new Vector3(Rm_SpawnPos_Laundry.x + Rm_Laundry.Width, Rm_SpawnPos_Laundry.y, Rm_SpawnPos_Laundry.z);

	//	List<Vector3> Rm_CornerList_Laundry = new List<Vector3> { Rm_Laundry_TopLeft, Rm_Laundry_TopRight, Rm_Laundry_BottomLeft, Rm_Laundry_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Laundry = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Laundry_TopLeft",		 1, Rm_Laundry_TopLeft),
	//		GenerateVertex("Vx_Laundry_TopRight",		 1, Rm_Laundry_TopRight),
	//		GenerateVertex("Vx_Laundry_BottomLeft",	 1, Rm_Laundry_BottomLeft),
	//		GenerateVertex("Vx_Laundry_BottomRight", 1, Rm_Laundry_BottomRight)
	//	};


	//	// 7 Rm_EntryWay_Vertices()
	//	Vector3 Rm_SpawnPos_EntryWay = new Vector3(Rm_Laundry_BottomRight.x, LevelOneHeight, Rm_Laundry_BottomRight.z);

	//	Vector3 Rm_EntryWay_TopLeft			= new Vector3(Rm_SpawnPos_EntryWay.x,											Rm_SpawnPos_EntryWay.y, Rm_SpawnPos_EntryWay.z + Rm_EntryWay.Length);
	//	Vector3 Rm_EntryWay_TopRight		= new Vector3(Rm_SpawnPos_EntryWay.x + Rm_EntryWay.Width, Rm_SpawnPos_EntryWay.y, Rm_SpawnPos_EntryWay.z + Rm_EntryWay.Length);
	//	Vector3 Rm_EntryWay_BottomLeft	= new Vector3(Rm_SpawnPos_EntryWay.x,											Rm_SpawnPos_EntryWay.y, Rm_SpawnPos_EntryWay.z);
	//	Vector3 Rm_EntryWay_BottomRight = new Vector3(Rm_SpawnPos_EntryWay.x + Rm_EntryWay.Width, Rm_SpawnPos_EntryWay.y, Rm_SpawnPos_EntryWay.z);

	//	List<Vector3> Rm_CornerList_EntryWay = new List<Vector3> { Rm_EntryWay_TopLeft, Rm_EntryWay_TopRight, Rm_EntryWay_BottomLeft, Rm_EntryWay_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_EntryWay = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_EntryWay_TopLeft",			1, Rm_EntryWay_TopLeft),
	//		GenerateVertex("Vx_EntryWay_TopRight",		1, Rm_EntryWay_TopRight),
	//		GenerateVertex("Vx_EntryWay_BottomLeft",	1, Rm_EntryWay_BottomLeft),
	//		GenerateVertex("Vx_EntryWay_BottomRight", 1, Rm_EntryWay_BottomRight)
	//	};


	//	// 8 Rm_EntryWayCloset_Vertices()
	//	Vector3 Rm_SpawnPos_EntryWayCloset = new Vector3(Rm_EntryWay_TopRight.x, LevelOneHeight, Rm_EntryWay_TopRight.z - Rm_EntryCloset.Length);

	//	Vector3 Rm_EntryWayCloset_TopLeft			= new Vector3(Rm_SpawnPos_EntryWayCloset.x,												 Rm_SpawnPos_EntryWayCloset.y, Rm_SpawnPos_EntryWayCloset.z + Rm_EntryCloset.Length);
	//	Vector3 Rm_EntryWayCloset_TopRight		= new Vector3(Rm_SpawnPos_EntryWayCloset.x + Rm_EntryCloset.Width, Rm_SpawnPos_EntryWayCloset.y, Rm_SpawnPos_EntryWayCloset.z + Rm_EntryCloset.Length);
	//	Vector3 Rm_EntryWayCloset_BottomLeft	= new Vector3(Rm_SpawnPos_EntryWayCloset.x,												 Rm_SpawnPos_EntryWayCloset.y, Rm_SpawnPos_EntryWayCloset.z);
	//	Vector3 Rm_EntryWayCloset_BottomRight = new Vector3(Rm_SpawnPos_EntryWayCloset.x + Rm_EntryCloset.Width, Rm_SpawnPos_EntryWayCloset.y, Rm_SpawnPos_EntryWayCloset.z);

	//	List<Vector3> Rm_CornerList_EntryWayCloset = new List<Vector3> { Rm_EntryWayCloset_TopLeft, Rm_EntryWayCloset_TopRight, Rm_EntryWayCloset_BottomLeft, Rm_EntryWayCloset_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_EntryWayCloset = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_EntryWayCloset_TopLeft",			1, Rm_EntryWayCloset_TopLeft),
	//		GenerateVertex("Vx_EntryWayCloset_TopRight",		1, Rm_EntryWayCloset_TopRight),
	//		GenerateVertex("Vx_EntryWayCloset_BottomLeft",	1, Rm_EntryWayCloset_BottomLeft),
	//		GenerateVertex("Vx_EntryWayCloset_BottomRight", 1, Rm_EntryWayCloset_BottomRight)
	//	};


	//	// 9 Rm_Bath_Vertices()
	//	Vector3 Rm_SpawnPos_Bath = new Vector3(Rm_Hallway_BottomRight.x - Rm_Bath.Width, LevelOneHeight, Rm_Hallway_BottomRight.z - Rm_Bath.Length);

	//	Vector3 Rm_Bath_TopLeft			= new Vector3(Rm_SpawnPos_Bath.x,									Rm_SpawnPos_Bath.y, Rm_SpawnPos_Bath.z + Rm_Bath.Length);
	//	Vector3 Rm_Bath_TopRight		= new Vector3(Rm_SpawnPos_Bath.x + Rm_Bath.Width, Rm_SpawnPos_Bath.y, Rm_SpawnPos_Bath.z + Rm_Bath.Length);
	//	Vector3 Rm_Bath_BottomLeft	= new Vector3(Rm_SpawnPos_Bath.x,									Rm_SpawnPos_Bath.y, Rm_SpawnPos_Bath.z);
	//	Vector3 Rm_Bath_BottomRight = new Vector3(Rm_SpawnPos_Bath.x + Rm_Bath.Width, Rm_SpawnPos_Bath.y, Rm_SpawnPos_Bath.z);

	//	List<Vector3> Rm_CornerList_Bath = new List<Vector3> { Rm_Bath_TopLeft, Rm_Bath_TopRight, Rm_Bath_BottomLeft, Rm_Bath_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Bath = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Bath_TopLeft",			1, Rm_Bath_TopLeft),
	//		GenerateVertex("Vx_Bath_TopRight",		1, Rm_Bath_TopRight),
	//		GenerateVertex("Vx_Bath_BottomLeft",	1, Rm_Bath_BottomLeft),
	//		GenerateVertex("Vx_Bath_BottomRight", 1, Rm_Bath_BottomRight)
	//	};


	//	// 10 Rm_BedCloset_Vertices()
	//	Vector3 Rm_SpawnPos_BedCloset = new Vector3(Rm_Bath_BottomRight.x - Rm_BedCloset.Width, LevelOneHeight, Rm_Bath_BottomRight.z - Rm_BedCloset.Length);

	//	Vector3 Rm_BedCloset_TopLeft			= new Vector3(Rm_SpawnPos_BedCloset.x,											Rm_SpawnPos_BedCloset.y, Rm_SpawnPos_BedCloset.z + Rm_BedCloset.Length);
	//	Vector3 Rm_BedCloset_TopRight			= new Vector3(Rm_SpawnPos_BedCloset.x + Rm_BedCloset.Width, Rm_SpawnPos_BedCloset.y, Rm_SpawnPos_BedCloset.z + Rm_BedCloset.Length);
	//	Vector3 Rm_BedCloset_BottomLeft		= new Vector3(Rm_SpawnPos_BedCloset.x,											Rm_SpawnPos_BedCloset.y, Rm_SpawnPos_BedCloset.z);
	//	Vector3 Rm_BedCloset_BottomRight	= new Vector3(Rm_SpawnPos_BedCloset.x + Rm_BedCloset.Width, Rm_SpawnPos_BedCloset.y, Rm_SpawnPos_BedCloset.z);

	//	List<Vector3> Rm_CornerList_BedCloset = new List<Vector3> { Rm_BedCloset_TopLeft, Rm_BedCloset_TopRight, Rm_BedCloset_BottomLeft, Rm_BedCloset_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_BedCloset = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_BedCloset_TopLeft",		 1, Rm_BedCloset_TopLeft),
	//		GenerateVertex("Vx_BedCloset_TopRight",		 1, Rm_BedCloset_TopRight),
	//		GenerateVertex("Vx_BedCloset_BottomLeft",	 1, Rm_BedCloset_BottomLeft),
	//		GenerateVertex("Vx_BedCloset_BottomRight", 1, Rm_BedCloset_BottomRight)
	//	};


	//	// 11 Rm_StoreEntry_Vertices()
	//	Vector3 Rm_SpawnPos_StoreEntry = new Vector3(Rm_BedCloset_BottomRight.x - Rm_StoreEntry.Width, LevelOneHeight, Rm_BedCloset_BottomRight.z - Rm_StoreEntry.Length);

	//	Vector3 Rm_StoreEntry_TopLeft			= new Vector3(Rm_SpawnPos_StoreEntry.x,												Rm_SpawnPos_StoreEntry.y, Rm_SpawnPos_StoreEntry.z + Rm_StoreEntry.Length);
	//	Vector3 Rm_StoreEntry_TopRight		= new Vector3(Rm_SpawnPos_StoreEntry.x + Rm_StoreEntry.Width, Rm_SpawnPos_StoreEntry.y, Rm_SpawnPos_StoreEntry.z + Rm_StoreEntry.Length);
	//	Vector3 Rm_StoreEntry_BottomLeft	= new Vector3(Rm_SpawnPos_StoreEntry.x,											  Rm_SpawnPos_StoreEntry.y, Rm_SpawnPos_StoreEntry.z);
	//	Vector3 Rm_StoreEntry_BottomRight	= new Vector3(Rm_SpawnPos_StoreEntry.x + Rm_StoreEntry.Width, Rm_SpawnPos_StoreEntry.y, Rm_SpawnPos_StoreEntry.z);

	//	List<Vector3> Rm_CornerList_StoreEntry = new List<Vector3> { Rm_StoreEntry_TopLeft, Rm_StoreEntry_TopRight, Rm_StoreEntry_BottomLeft, Rm_StoreEntry_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_StoreEntry = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_StoreEntry_TopLeft",		  1, Rm_StoreEntry_TopLeft),
	//		GenerateVertex("Vx_StoreEntry_TopRight",		1, Rm_StoreEntry_TopRight),
	//		GenerateVertex("Vx_StoreEntry_BottomLeft",	1, Rm_StoreEntry_BottomLeft),
	//		GenerateVertex("Vx_StoreEntry_BottomRight", 1, Rm_StoreEntry_BottomRight)
	//	};


	//	// 12 Rm_Store_Vertices()
	//	Vector3 Rm_SpawnPos_Store = new Vector3(Rm_StoreEntry_BottomLeft.x - Rm_Store.Width, LevelOneHeight, Rm_StoreEntry_BottomLeft.z);

	//	Vector3 Rm_Store_TopLeft		 = new Vector3(Rm_SpawnPos_Store.x,									 Rm_SpawnPos_Store.y, Rm_SpawnPos_Store.z + Rm_Store.Length);
	//	Vector3 Rm_Store_TopRight		 = new Vector3(Rm_SpawnPos_Store.x + Rm_Store.Width, Rm_SpawnPos_Store.y, Rm_SpawnPos_Store.z + Rm_Store.Length);
	//	Vector3 Rm_Store_BottomLeft	 = new Vector3(Rm_SpawnPos_Store.x,									 Rm_SpawnPos_Store.y, Rm_SpawnPos_Store.z);
	//	Vector3 Rm_Store_BottomRight = new Vector3(Rm_SpawnPos_Store.x + Rm_Store.Width, Rm_SpawnPos_Store.y, Rm_SpawnPos_Store.z);

	//	List<Vector3> Rm_CornerList_Store = new List<Vector3> { Rm_Store_TopLeft, Rm_Store_TopRight, Rm_Store_BottomLeft, Rm_Store_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Store = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Store_TopLeft",		 1, Rm_Store_TopLeft),
	//		GenerateVertex("Vx_Store_TopRight",		 1, Rm_Store_TopRight),
	//		GenerateVertex("Vx_Store_BottomLeft",  1, Rm_Store_BottomLeft),
	//		GenerateVertex("Vx_Store_BottomRight", 1, Rm_Store_BottomRight)
	//	};



	//	// 13 Rm_DeckCovered_Vertices()
	//	Vector3 Rm_SpawnPos_DeckCovered = new Vector3(Rm_Living_BottomLeft.x, LevelOneHeight, Rm_Living_BottomLeft.z - Rm_DeckCovered.Length);

	//	Vector3 Rm_DeckCovered_TopLeft			= new Vector3(Rm_SpawnPos_DeckCovered.x,												Rm_SpawnPos_DeckCovered.y, Rm_SpawnPos_DeckCovered.z + Rm_DeckCovered.Length);
	//	Vector3	Rm_DeckCovered_TopRight			= new Vector3(Rm_SpawnPos_DeckCovered.x + Rm_DeckCovered.Width, Rm_SpawnPos_DeckCovered.y, Rm_SpawnPos_DeckCovered.z + Rm_DeckCovered.Length);
	//	Vector3 Rm_DeckCovered_BottomLeft		= new Vector3(Rm_SpawnPos_DeckCovered.x,												Rm_SpawnPos_DeckCovered.y, Rm_SpawnPos_DeckCovered.z);
	//	Vector3 Rm_DeckCovered_BottomRight	= new Vector3(Rm_SpawnPos_DeckCovered.x + Rm_DeckCovered.Width, Rm_SpawnPos_DeckCovered.y, Rm_SpawnPos_DeckCovered.z);

	//	List<Vector3> Rm_CornerList_DeckCovered = new List<Vector3> { Rm_DeckCovered_TopLeft, Rm_DeckCovered_TopRight, Rm_DeckCovered_BottomLeft, Rm_DeckCovered_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_DeckCovered = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_DeckCovered_TopLeft",		 1, Rm_DeckCovered_TopLeft),
	//		GenerateVertex("Vx_DeckCovered_TopRight",		 1, Rm_DeckCovered_TopRight),
	//		GenerateVertex("Vx_DeckCovered_BottomLeft",  1, Rm_DeckCovered_BottomLeft),
	//		GenerateVertex("Vx_DeckCovered_BottomRight", 1, Rm_DeckCovered_BottomRight)
	//	};



	//	// 14 Rm_DeckUncovered_Vertices()
	//	Vector3 Rm_SpawnPos_DeckUncovered = new Vector3(Rm_SpawnPos_DeckCovered.x, LevelOneHeight, Rm_SpawnPos_DeckCovered.z - Rm_DeckUncovered.Length);

	//	Vector3 Rm_DeckUncovered_TopLeft		 = new Vector3(Rm_SpawnPos_DeckUncovered.x,													 Rm_SpawnPos_DeckUncovered.y, Rm_SpawnPos_DeckUncovered.z + Rm_DeckUncovered.Length);
	//	Vector3	Rm_DeckUncovered_TopRight		 = new Vector3(Rm_SpawnPos_DeckUncovered.x + Rm_DeckUncovered.Width, Rm_SpawnPos_DeckUncovered.y, Rm_SpawnPos_DeckUncovered.z + Rm_DeckUncovered.Length);
	//	Vector3 Rm_DeckUncovered_BottomLeft	 = new Vector3(Rm_SpawnPos_DeckUncovered.x,													 Rm_SpawnPos_DeckUncovered.y, Rm_SpawnPos_DeckUncovered.z);
	//	Vector3 Rm_DeckUncovered_BottomRight = new Vector3(Rm_SpawnPos_DeckUncovered.x + Rm_DeckUncovered.Width, Rm_SpawnPos_DeckUncovered.y, Rm_SpawnPos_DeckUncovered.z);

	//	List<Vector3> Rm_CornerList_DeckUncovered = new List<Vector3> { Rm_DeckUncovered_TopLeft, Rm_DeckUncovered_TopRight, Rm_DeckUncovered_BottomLeft, Rm_DeckUncovered_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_DeckUncovered = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_DeckUncovered_TopLeft",		 1, Rm_DeckUncovered_TopLeft),
	//		GenerateVertex("Vx_DeckUncovered_TopRight",		 1, Rm_DeckUncovered_TopRight),
	//		GenerateVertex("Vx_DeckUncovered_BottomLeft",  1, Rm_DeckUncovered_BottomLeft),
	//		GenerateVertex("Vx_DeckUncovered_BottomRight", 1, Rm_DeckUncovered_BottomRight)
	//	};



	//	// 15 Rm_Loft_Vertices()
	//	Vector3 Rm_SpawnPos_Loft = new Vector3(Rm_Dining_TopLeft.x, LevelTwoHeight, Rm_Dining_TopLeft.z - Rm_Loft.Length);			// Loft is on the second floor, so we add height

	//	Vector3 Rm_Loft_TopLeft			= new Vector3(Rm_SpawnPos_Loft.x,									Rm_SpawnPos_Loft.y, Rm_SpawnPos_Loft.z + Rm_Loft.Length);
	//	Vector3 Rm_Loft_TopRight		= new Vector3(Rm_SpawnPos_Loft.x + Rm_Loft.Width, Rm_SpawnPos_Loft.y, Rm_SpawnPos_Loft.z + Rm_Loft.Length);
	//	Vector3 Rm_Loft_BottomLeft  = new Vector3(Rm_SpawnPos_Loft.x,									Rm_SpawnPos_Loft.y, Rm_SpawnPos_Loft.z);
	//	Vector3 Rm_Loft_BottomRight = new Vector3(Rm_SpawnPos_Loft.x + Rm_Loft.Width, Rm_SpawnPos_Loft.y, Rm_SpawnPos_Loft.z);

	//	List<Vector3> Rm_CornerList_Loft = new List<Vector3> { Rm_Loft_TopLeft, Rm_Loft_TopRight, Rm_Loft_BottomLeft, Rm_Loft_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_Loft = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_Loft_TopLeft",			2, Rm_Loft_TopLeft),
	//		GenerateVertex("Vx_Loft_TopRight",		2, Rm_Loft_TopRight),
	//		GenerateVertex("Vx_Loft_BottomLeft",  2, Rm_Loft_BottomLeft),
	//		GenerateVertex("Vx_Loft_BottomRight", 2, Rm_Loft_BottomRight)
	//	};


	//	// 16 Rm_UpperBedEntry_Vertices()
	//	Vector3 Rm_SpawnPos_UpperBedEntry = new Vector3(Rm_Loft_TopRight.x, LevelTwoHeight, Rm_Loft_TopRight.z - Rm_UpperBedEntry.Length);

	//	Vector3 Rm_UpperBedEntry_TopLeft		 = new Vector3(Rm_SpawnPos_UpperBedEntry.x,													 Rm_SpawnPos_UpperBedEntry.y, Rm_SpawnPos_UpperBedEntry.z + Rm_UpperBedEntry.Length);
	//	Vector3 Rm_UpperBedEntry_TopRight		 = new Vector3(Rm_SpawnPos_UpperBedEntry.x + Rm_UpperBedEntry.Width, Rm_SpawnPos_UpperBedEntry.y, Rm_SpawnPos_UpperBedEntry.z + Rm_UpperBedEntry.Length);
	//	Vector3 Rm_UpperBedEntry_BottomLeft  = new Vector3(Rm_SpawnPos_UpperBedEntry.x,													 Rm_SpawnPos_UpperBedEntry.y, Rm_SpawnPos_UpperBedEntry.z);
	//	Vector3 Rm_UpperBedEntry_BottomRight = new Vector3(Rm_SpawnPos_UpperBedEntry.x + Rm_UpperBedEntry.Width, Rm_SpawnPos_UpperBedEntry.y, Rm_SpawnPos_UpperBedEntry.z);

	//	List<Vector3> Rm_CornerList_UpperBedEntry = new List<Vector3> { Rm_UpperBedEntry_TopLeft, Rm_UpperBedEntry_TopRight, Rm_UpperBedEntry_BottomLeft, Rm_UpperBedEntry_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_UpperBedEntry = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_UpperBedEntry_TopLeft",		 2, Rm_UpperBedEntry_TopLeft),
	//		GenerateVertex("Vx_UpperBedEntry_TopRight",		 2, Rm_UpperBedEntry_TopRight),
	//		GenerateVertex("Vx_UpperBedEntry_BottomLeft",  2, Rm_UpperBedEntry_BottomLeft),
	//		GenerateVertex("Vx_UpperBedEntry_BottomRight", 2, Rm_UpperBedEntry_BottomRight)
	//	};


	//	// 17 Rm_UpperBed_Vertices()
	//	Vector3 Rm_SpawnPos_UpperBed = new Vector3(Rm_UpperBedEntry_BottomLeft.x, LevelTwoHeight, Rm_UpperBedEntry_BottomLeft.z - Rm_UpperBed.Length);

	//	Vector3 Rm_UpperBed_TopLeft			= new Vector3(Rm_SpawnPos_UpperBed.x,											Rm_SpawnPos_UpperBed.y, Rm_SpawnPos_UpperBed.z + Rm_UpperBed.Length);
	//	Vector3 Rm_UpperBed_TopRight		= new Vector3(Rm_SpawnPos_UpperBed.x + Rm_UpperBed.Width, Rm_SpawnPos_UpperBed.y, Rm_SpawnPos_UpperBed.z + Rm_UpperBed.Length);
	//	Vector3 Rm_UpperBed_BottomLeft	= new Vector3(Rm_SpawnPos_UpperBed.x,											Rm_SpawnPos_UpperBed.y, Rm_SpawnPos_UpperBed.z);
	//	Vector3 Rm_UpperBed_BottomRight = new Vector3(Rm_SpawnPos_UpperBed.x + Rm_UpperBed.Width, Rm_SpawnPos_UpperBed.y, Rm_SpawnPos_UpperBed.z);

	//	List<Vector3> Rm_CornerList_UpperBed = new List<Vector3> { Rm_UpperBed_TopLeft, Rm_UpperBed_TopRight, Rm_UpperBed_BottomLeft, Rm_UpperBed_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_UpperBed = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_UpperBed_TopLeft",		  2, Rm_UpperBed_TopLeft),
	//		GenerateVertex("Vx_UpperBed_TopRight",		2, Rm_UpperBed_TopRight),
	//		GenerateVertex("Vx_UpperBed_BottomLeft",  2, Rm_UpperBed_BottomLeft),
	//		GenerateVertex("Vx_UpperBed_BottomRight", 2, Rm_UpperBed_BottomRight)
	//	};


	//	// 18 Rm_UpperBedCloset_Vertices()
	//	Vector3 Rm_SpawnPos_UpperBedCloset		= new Vector3(Rm_UpperBed_BottomRight.x, LevelTwoHeight, Rm_UpperBed_BottomRight.z);

	//	Vector3 Rm_UpperBedCloset_TopLeft			= new Vector3(Rm_SpawnPos_UpperBedCloset.x,													  Rm_SpawnPos_UpperBedCloset.y, Rm_SpawnPos_UpperBedCloset.z + Rm_UpperBedCloset.Length);
	//	Vector3 Rm_UpperBedCloset_TopRight		= new Vector3(Rm_SpawnPos_UpperBedCloset.x + Rm_UpperBedCloset.Width, Rm_SpawnPos_UpperBedCloset.y, Rm_SpawnPos_UpperBedCloset.z + Rm_UpperBedCloset.Length);
	//	Vector3 Rm_UpperBedCloset_BottomLeft	= new Vector3(Rm_SpawnPos_UpperBedCloset.x,													  Rm_SpawnPos_UpperBedCloset.y, Rm_SpawnPos_UpperBedCloset.z);
	//	Vector3 Rm_UpperBedCloset_BottomRight	= new Vector3(Rm_SpawnPos_UpperBedCloset.x + Rm_UpperBedCloset.Width, Rm_SpawnPos_UpperBedCloset.y, Rm_SpawnPos_UpperBedCloset.z);

	//	List<Vector3> Rm_CornerList_UpperBedCloset = new List<Vector3> { Rm_UpperBedCloset_TopLeft, Rm_UpperBedCloset_TopRight, Rm_UpperBedCloset_BottomLeft, Rm_UpperBedCloset_BottomRight };

	//	List<Rm_Vertex> Rm_VertexList_UpperBedCloset = new List<Rm_Vertex> {
	//		GenerateVertex("Vx_UpperBedCloset_TopLeft",			2, Rm_UpperBedCloset_TopLeft),
	//		GenerateVertex("Vx_UpperBedCloset_TopRight",		2, Rm_UpperBedCloset_TopRight),
	//		GenerateVertex("Vx_UpperBedCloset_BottomLeft",	2, Rm_UpperBedCloset_BottomLeft),
	//		GenerateVertex("Vx_UpperBedCloset_BottomRight", 2, Rm_UpperBedCloset_BottomRight)
	//	};



	//	// 19 All_Room_Vertices()

	//	List<List<Vector3>> Rm_All_CornerList = new List<List<Vector3>>();
	//	Rm_All_CornerList.AddRange(new List<List<Vector3>> { 
	//		Rm_CornerList_Living, 
	//		Rm_CornerList_Dining, 
	//		Rm_CornerList_Hallway, 
	//		Rm_CornerList_Bed, 
	//		Rm_CornerList_Kitchen, 
	//		Rm_CornerList_Laundry, 
	//		Rm_CornerList_EntryWay, 
	//		Rm_CornerList_EntryWayCloset, 
	//		Rm_CornerList_Bath, 
	//		Rm_CornerList_BedCloset, 
	//		Rm_CornerList_StoreEntry, 
	//		Rm_CornerList_Store, 
	//		Rm_CornerList_DeckCovered, 
	//		Rm_CornerList_DeckUncovered, 
	//		Rm_CornerList_Loft, 
	//		Rm_CornerList_UpperBedEntry, 
	//		Rm_CornerList_UpperBed, 
	//		Rm_CornerList_UpperBedCloset 
	//	});

	#endregion




	#region Measurements


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

	#endregion

	//20260801 Measurements for each room, including walls, doors, and windows. These measurements are used to create the floor plan and to calculate the square footage of each room.
	#region Measurement Save Before Adjustments


	//// ----- ----- ----- ----- ----- ----- ----- Initializer ----- ----- ----- ----- ----- ----- -----

	//public static FloorSection Rm_Living = new FloorSection("LivingRoom", "Rm_Living", 1, 151.75f, 152f, "LVP");
	//public static FloorSection Rm_Dining = new FloorSection("DiningRoom", "Rm_Dining", 1, 125.75f, 95.25f, "LVP");
	//public static FloorSection Rm_Hallway = new FloorSection("Hallway", "Rm_Hallway", 1, 229f, 36f, "LVP");
	//public static FloorSection Rm_Bed = new FloorSection("Bedroom", "Rm_Bed", 1, 157f, 148f, "LVP");
	//public static FloorSection Rm_Kitchen = new FloorSection("Kitchen", "Rm_Kitchen", 1, 114f, 90.5f, "LVP");

	//public static FloorSection Rm_Laundry = new FloorSection("Laundry", "Rm_Laundry", 1, 35.75f, 89.5f, "LVP");
	//public static FloorSection Rm_EntryWay = new FloorSection("EntryWay", "Rm_EntryWay", 1, 43f, 40.25f, "LVP");
	//public static FloorSection Rm_EntryCloset = new FloorSection("EntryCloset", "Rm_EntryCloset", 1, 57f, 35f, "LVP");
	//public static FloorSection Rm_Bath = new FloorSection("Bathroom", "Rm_Bath", 1, 71.25f, 87f, "LVP");
	//public static FloorSection Rm_BedCloset = new FloorSection("BedroomCloset", "Rm_BedCloset", 1, 71f, 78f, "LVP");

	//public static FloorSection Rm_StoreEntry = new FloorSection("StoreroomEntry", "Rm_StoreEntry", 1, 65.5f, 103f, "Carpet");
	//public static FloorSection Rm_Store = new FloorSection("Storeroom", "Rm_Store", 1, 120f, 127.5f, "Carpet");
	//public static FloorSection Rm_DeckCovered = new FloorSection("DeckCovered", "Rm_DeckCovered", 1, 158f, 40f, "Rubber");
	//public static FloorSection Rm_DeckUncovered = new FloorSection("DeckUncovered", "Rm_DeckUncovered", 1, 185f, 28f, "Rubber");

	//// 2nd Floor
	//public static FloorSection Rm_Loft = new FloorSection("Loft", "Rm_Loft", 2, 183f, 137f, "Carpet");
	//public static FloorSection Rm_UpperBed = new FloorSection("UpperBedroom", "Rm_UpperBed", 2, 165f, 76f, "Carpet");
	//public static FloorSection Rm_UpperBedEntry = new FloorSection("UpperBedroomEntry", "Rm_UpperBedEntry", 2, 87f, 54.75f, "Carpet");
	//public static FloorSection Rm_UpperBedCloset = new FloorSection("UpperBedroomCloset", "Rm_UpperBedCloset", 2, 23.75f, 76f, "Carpet");

	#endregion

}

