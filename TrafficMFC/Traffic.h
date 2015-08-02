#include "helper_math.h"

namespace Traffic {
	class TrafficLight;
	class NetworkLayer;
	class NodeConnetion;
	class LineNode;
	class LineSegment;
	class IVehicle;
	class Intersections;

	class IVehicle {
	public:
		void Think();
		void Move();
		void Reset();
	};

	class TrafficLight {

	};

	class NetworkLayer {

	};

	template<class T>
	class Interval {
	public:
		T left;
		T right;
		Interval(T start, T end);
		bool Contains(T value);
		bool IntersectTrue(Interval<T> otherInterval);
	};

	struct CrossingVehicleTimes {
		double originalArrivingTime;
		double remainingDistance;
		bool willWaitInFrontOfIntersection;
		Interval<double> blockingTime;
		CrossingVehicleTimes(double originalArrivingTime, double remainingDistance, Interval<double> blockingTime, bool willWaitInFrontOfIntersection);
	};

	class Intersection {
	public:
		double _frontWaitingDistance;
		double _rearWaitingDistance;

		NodeConnection *_aConnection;
		Intersection **_aListNode;
		double _aTime;
		double2 aPosition;
		double2 getAPosition();
		double getAArcPosition();

		NodeConnection *_bConnection;
		Intersection **_bListNode;
		double _bTime;
		double2 bPosition;
		double2 getBPosition();
		double getBArcPosition();

		bool avoidBlocking;
		bool getAvoidBlocking();

		Intersection(NodeConnection *_aConnection, NodeConnection *_bConnection, double _aTime, double _bTime);
		double GetWaitingDistance();
		bool ContainsNodeConnection(NodeConnection *nc);
		void Dispose();
		NodeConnection *GetOtherNodeConnection(NodeConnection *nc);
		double GetMyTime(NodeConnection *nc);
		double GetMyArcPosition(NodeConnection *nc);
		Intersection **GetMyListNode(NodeConnection *nc);

		IVehicle *aCrossingVehicles;
		CrossingVehicleTimes *aCrossingVehicleTime;

		IVehicle *bCrossingVehicles;
		CrossingVehicleTimes *bCrossingVehicleTime;

		void RegisterVehicle(IVehicle v, NodeConnection nc, double distance, double currentTime);
		void UpdateVehicle(IVehicle v, NodeConnection nc, double distance, double currentTime);
		void UpdateVehicle(IVehicle v, NodeConnection nc, bool willWaitInFrontOfIntersection);
		void UnregisterVehicle(IVehicle v, NodeConnection nc);
		void UnregisterAllVehicles();
		CrossingVehicleTimes *CalculateInterferingVehicles(IVehicle v, NodeConnection nc);
		CrossingVehicleTimes GetCrossingVehicleTimes(IVehicle v, NodeConnection nc);

		bool calculatedInterferingVehicles = false;
		double CalculateArrivingTime(IVehicle v, double distance);
	};

	struct SpecificIntersection {
		NodeConnection *nodeConnection;
		Intersection *intersection;
		SpecificIntersection(NodeConnection *nc, Intersection *i);
		bool operator == (SpecificIntersection & other);
	};

	class VehicleDistance {
		IVehicle *vehicle;
		double distance;
		VehicleDistance(IVehicle *vehicle, double distance);
		static VehicleDistance Min(VehicleDistance &lhs, VehicleDistance &rhs);
		static VehicleDistance Max(VehicleDistance &lhs, VehicleDistance &rhs);
		static VehicleDistance MinTail(VehicleDistance &lhs, VehicleDistance &rhs);
		static bool operator < (VehicleDistance &lhs, VehicleDistance &rhs);
		static bool operator > (VehicleDistance &lhs, VehicleDistance &rhs);
	};

	class NodeConnection {
	public:
		struct SpecificPosition {
		public:
			NodeConnection *nc;
			double time;
			double arcPosition;
			SpecificPosition(NodeConnection nc, double time);
			SpecificPosition(double arcPosition, NodeConnection nc);
		};

		struct LineChangePoint {
			SpecificPosition start;
			SpecificPosition target;
			SpecificPosition otherStart;
			double parallelDistance;
			LineSegment lineSegment;
			double getLength();
		};

		bool cmp(LineChangePoint *a, LineChangePoint *b) {

		}

		struct LineChangeInterval {
		public:
			LineNode targetNode;
			double startArcPos;
			double endArcPos;
			Interval<double> arcPosInterval;
			double getLength();
			LineChangeInterval(LineNode targetNode, double startArcPos, double endArcPos);
		};

	public:
		LineNode *startNode;
		LineNode *endNode;
		LineSegment *lineSegment;
		int priority;

		bool enableOutgoingLineChange;
		bool enableIncomingLineChange;
		double targetVelocity;

		LineChangePoint *lineChangePoints;
		LineChangeInterval *lineChangeIntervals;
		LineNode *viaLineChangeReachableNodes;

		Intersection **intersections;
		void AddIntersection(Intersection *i);
		void RemoveIntersection(Intersection *i);
		Intersection *GetIntersectionsWithinTime(Interval<double> interval);
		SpecificIntersection *GetIntersectionsWithinArcLength(Interval<double> interval);
		SpecificIntersection *GetSortedIntersectionsWithinArcLength(Interval<double> interval);

		int numVehicles;
		IVehicle *vehicles;
		int numVehiclesToRemove;
		IVehicle *vehiclesToRemove;
		bool CanSpawnVehicleAt(double arcPosition);
		bool AddVehicle(IVehicle *veh);
		void AddVehicleAt(IVehicle *v, double arcPosition);
		void RemoveVehicle(IVehicle *v);
		void RemoveAllVehiclesInRemoveList();
		IVehicle *GetVehicleListNodeBehindArcPosition(double arcPosition);
		IVehicle *GetVehicleListNodeBeforeArcPosition(double arcPosition);
		VehicleDistance GetVehicleBehindArcPosition(double arcPosition, double distanceWithin);
		VehicleDistance GetVehicleBeforeArcPosition(double arcPosition, double distanceWithin);
		VehicleDistance *GetVehiclesAroundArcPosition(double arcPosition, double distanceWithin);

		// an event here
		//void Handler_VehicleLeftNodeConnection(object sender, IVehicle.VehicleLeftNodeConnectionEventArgs e)

		int startNodeHash;
		int endNodeHash;
		LineNode *GetLineNodeByHash(LineNode *nodesList, int hash);

		void RemoveAllVehiclesInRemoveList();
		double GetLengthToLineNodeViaLineChange(LineNode *ln);


	};

	class LineSegment {
	public:
		double2 p0, p1, p2, p3;
		double length;
		LineSegment(double arcPosStart, double2 p0, double2 p1, double2 p2, double2 p3) {

		}
	};

	class LineNode {
	private:
		double2 _in;
		double2 _out;
		double2 _position;
	public:
		TrafficLight tLight;
		bool stopSign;
		double2 getPosition();
		void setPosition(double2 value);
		double2 getInSlope();
		void setInSlope(double2 value);
		double2 getInSlopeAbs();
		void setInSlopeAbs(double2 value);
		double2 getOutSlope();
		void setOutSlope(double2 value);
		double2 getOutSlopeAbs();
		void setOutSlopeAbs(double2 value);

		double2 getPosition() {
			return _position;
		}
		void setPosition(double2 value) {
			_position = value; UpdateNodeConnections(true); UpdateNodeGraphics();
		}
		double2 getInSlope() {
			return _in;
		}
		void setInSlope(double2 value){
			_in = value; UpdateNodeConnections(true); UpdateNodeGraphics();
		}
		double2 getInSlopeAbs() {
			return _in + _position;
		}
		void setInSlopeAbs(double2 value){
			_in = value - _position; UpdateNodeConnections(true); UpdateNodeGraphics();
		}
		double2 getOutSlope() {
			return _out;
		}
		void setOutSlope(double2 value){
			_out = value; UpdateNodeConnections(true); UpdateNodeGraphics();
		}
		double2 getOutSlopeAbs() {
			return _out + _position;
		}
		void setOutSlopeAbs(double2 value){
			_out = value - _position; UpdateNodeConnections(true); UpdateNodeGraphics();
		}

		NetworkLayer *networkLayer;
		int numNextConnections, numPrevConnections;
		NodeConnection **nextConnections;
		NodeConnection **prevConnections;

		NodeConnection *GetNodeConnectionTo(LineNode *lineNode);

		NodeConnection *GetNodeConnectionTo(LineNode *lineNode) {
			for (int i = 0; i < numNextConnections; i++)
				if (lineNode == nextConnections[i]->endNode)
					return nextConnections[i];
			return nullptr;
		}

		void UpdateNodeConnections(bool doRecursive) {
			for (int i = 0; i < numNextConnections; i++) {
				NodeConnection *lc = nextConnections[i];
				delete lc->lineSegment;
				lc->lineSegment = new LineSegment(0, getPosition(), getOutSlopeAbs(),
					lc->endNode->getInSlopeAbs(), lc->endNode->getPosition());
			}
			if (doRecursive) {
				for (int i = 0; i < numPrevConnections; i++) {
					NodeConnection *lc = prevConnections[i];
					lc->startNode->UpdateNodeConnections(false);
				}
			}
		}

		LineNode(double2 position, NetworkLayer *nl, bool stopSign)
		{
			setPosition(double2());
			setInSlope(double2());
			setOutSlope(double2());
			networkLayer = nl;
			stopSign = stopSign;

			// Hashcode vergeben
			//InitStopSignEdgeOffsets();
		}

		void Tick(double tickLength) {
			for (int i = 0; i < numNextConnections; i++) {
				NodeConnection *nc = nextConnections[i];
				for (int j = 0; j < nc->numVehicles; j++) {
					IVehicle &v = nc->vehicles[j];
					v.Think();
				}
			}
			for (int i = 0; i < numNextConnections; i++) {
				NodeConnection *nc = nextConnections[i];
				nc->RemoveAllVehiclesInRemoveList();
				for (int j = 0; j < nc->numVehicles; j++) {
					IVehicle &v = nc->vehicles[j];
					v.Move();
				}
				nc->RemoveAllVehiclesInRemoveList();
			}
		}

		void Reset() {
			for (int i = 0; i < numNextConnections; i++) {
				NodeConnection *nc = nextConnections[i];
				for (int j = 0; j < nc->numVehicles; j++) {
					IVehicle &v = nc->vehicles[j];
					v.Reset();
				}
			}
		}

		void UpdateNodeConnections(bool doRecursive);
		void UpdateNodeGraphics();

		class LinkedLineNode {
		public:
			LineNode *node;
			LinkedLineNode *m_parent;
			LinkedLineNode *parent;
			bool lineChangeNeeded;
			int countOfParents;
			double length;

			void setParent(LinkedLineNode *value) {
				m_parent = value;
				if (m_parent == nullptr)
					countOfParents = 0;
				else
					countOfParents = m_parent->countOfParents + 1;

				length = (parent == nullptr) ? 0 : parent->length;
				if (parent != nullptr) {
					if (!lineChangeNeeded)
						length += parent->node->GetNodeConnectionTo(node)->lineSegment->length;
					else {
						if (parent->node->numNextConnections > 0) {
							double minV = parent->node->nextConnections[0]->GetLengthToLineNodeViaLineChange(node);
							for (int i = 1; i < parent->node->numNextConnections; ++i)
								minV = min(minV, parent->node->nextConnections[i]->GetLengthToLineNodeViaLineChange(node));
						}
					}
				}
			}

			LinkedLineNode(LineNode *node, LinkedLineNode *parent, bool lineChangeNeeded) {
				this->node = node;
				this->parent = parent;
				this->lineChangeNeeded = lineChangeNeeded;
			}
		};
	};
}