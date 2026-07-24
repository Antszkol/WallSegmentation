#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <ogx/Math/Ransac.h>
#include <queue>
#include <algorithm>
#include <limits> 
#include "Utils.h"
#include "EasyMethodExtension.h"

using namespace ogx;
using namespace std;

// Struct for calculating walls' width and height
struct WallDimensions {
	double Height{ 0.0 };
	double Width{ 0.0 };

	void CalculateWallDims(const vector<pair<int, Math::Point3D>>& Candidates, const vector<float>& WallLabels, const vector<bool>& Visited, int WallId) {
		// Max values to make sure they overwrite points' coordinates
		double MinX = numeric_limits<double>::max(); double MaxX = -numeric_limits<double>::max();
		double MinY = numeric_limits<double>::max(); double MaxY = -numeric_limits<double>::max();
		double MinZ = numeric_limits<double>::max(); double MaxZ = -numeric_limits<double>::max();

		for (const auto& Pt : Candidates) {
			if (Visited[Pt.first] && static_cast<int>(WallLabels[Pt.first]) == WallId) {
				MinX = min(MinX, Pt.second.x()); MaxX = max(MaxX, Pt.second.x());
				MinY = min(MinY, Pt.second.y()); MaxY = max(MaxY, Pt.second.y());
				MinZ = min(MinZ, Pt.second.z()); MaxZ = max(MaxZ, Pt.second.z());
			}
		}
		Height = MaxZ - MinZ;
		Width = max(MaxX - MinX, MaxY - MinY);
	}
};

struct FinalFilter : public EasyMethodExtension
{
	Data::ResourceID NodeId;

	// Default param values for the algorithm
	double NormalThresholdPoints{ 0.2 };
	double NormalThresholdWall{ 0.05 };
	double PlaneThickness{ 0.25 };
	double BfsRadius{ 0.8 };
	double IsolatedRadius{ 0.8 };
	double NormalRadius{ 0.2 };
	double PlaneBaseRadius{ 10.0 };
	double MinWallHeight{ 8.0 };
	double MinWallWidth{ 2.0 };
	int RemovePercent{ 50 };
	int MinNeighbours{ 20 };
	int RansacIterations{ 300 };
	int MinInliersCount{ 6400 };
	bool RunDetailBfs{ true };
	double DetailBfsRadius{ 0.6 };
	int ScalesCount{ 3 };

	vector<float> WallLabels;
	vector<bool> Visited;

	FinalFilter() : EasyMethodExtension(L"Find vertical surfaces.")
	{
	}

	// Params definitions
	virtual void DefineParameters(ParameterBank& Bank)
	{
		Bank.Add(L"NodeId", NodeId).AsNode();
		Bank.Add(L"NormalTresholdPoints", NormalThresholdPoints);
		Bank.Add(L"NormalTresholdWall", NormalThresholdWall);
		Bank.Add(L"IsolatedRadius", IsolatedRadius);
		Bank.Add(L"BFSRadius", BfsRadius);
		Bank.Add(L"PlaneRadius", PlaneBaseRadius);
		Bank.Add(L"RANSACIterations", RansacIterations);
		Bank.Add(L"MinNeighbour", MinNeighbours);
		Bank.Add(L"PlaneThickness", PlaneThickness);
		Bank.Add(L"NormalFilterRadius", NormalRadius);
		Bank.Add(L"removePercent", RemovePercent);
		Bank.Add(L"minInliersCount", MinInliersCount);

		Bank.Add(L"ScalesCount", ScalesCount);
		Bank.Add(L"addBFS_Enable", RunDetailBfs);
		Bank.Add(L"addBFS_Radius", DetailBfsRadius);

		Bank.Add(L"MinWallHeight", MinWallHeight);
		Bank.Add(L"MinWallWidth", MinWallWidth);
	}

	void UnderSample(Data::Clouds::PointsRange& Points) {
		if (Points.size() == 0) return;

		std::vector<Data::Clouds::State> States;
		Points.GetStates(States);
		int Counter = 0;

		// clamp user's input
		int GuardedRemovePercent = clamp(RemovePercent, 0, 100);

		for (size_t I = 0; I < Points.size(); I++) {
			if (rand() % 100 < GuardedRemovePercent) {
				States[I].set(Data::Clouds::PS_DELETED);
				Counter++;
			}
		}
		Points.SetStates(States);
		OGX_LINE.Msg(ogx::Level::Warning, L"Undersampling removed " + to_wstring(Counter));
	}

	void FilterIsolatedPoints(Data::Clouds::ICloud* Cloud, Data::Clouds::PointsRange& AllPointsRange) {
		if (!Cloud || AllPointsRange.size() == 0) return;

		std::vector<Data::Clouds::State> States;
		AllPointsRange.GetStates(States);

		Data::Clouds::PointsRange NeighborRange;
		int Idx = 0;
		int RemovedCount = 0;

		for (const auto& Point : Data::Clouds::RangeLocalXYZConst(AllPointsRange))
		{
			if (States[Idx].test(Data::Clouds::PS_DELETED)) {
				++Idx;
				continue;
			}

			NeighborRange.clear();
			auto Kernel = Data::Clouds::SphericalSearchKernel(Math::Sphere3D(IsolatedRadius, Point.cast<double>()));
			Cloud->GetAccess().FindPoints(Kernel, NeighborRange);

			// check if neighbours amount is lower than the treshold
			if (static_cast<int>(NeighborRange.size()) < MinNeighbours)
			{
				States[Idx].set(Data::Clouds::PS_DELETED);
				++RemovedCount;
			}
			++Idx;
		}

		AllPointsRange.SetStates(States);
		Cloud->GetAccess().DeletePoints();

		OGX_LINE.Msg(Level::Info, L"Isolated point filter removed " + to_wstring(RemovedCount) + L" points.");
	}

	void BfsRegionGrowing(vector<pair<int, Math::Point3D>>& Candidates, pair<int, Math::Point3D> StartPoint, Math::Plane3D& Plane, int WallId) {
		if (Candidates.empty()) return;

		queue<pair<int, Math::Point3D>> BfsQueue;
		BfsQueue.push(StartPoint);
		Visited[StartPoint.first] = true;
		WallLabels[StartPoint.first] = WallId;

		while (!BfsQueue.empty()) {
			auto CurrentCandidate = BfsQueue.front();
			BfsQueue.pop();

			for (auto& Para : Candidates) {
				if (!Visited[Para.first]) {
					double SpatialDist = (Para.second - CurrentCandidate.second).norm();
					if (SpatialDist < BfsRadius &&
						Math::CalcPointToPlaneDistance3D(Para.second, Plane) < PlaneThickness) {
						BfsQueue.push(Para);
						WallLabels[Para.first] = WallId;
						Visited[Para.first] = true;
					}
				}
			}
		}
	}

	void NormalFilter(Data::Clouds::ICloud* Cloud, Data::Clouds::PointsRange& PointsRangeObj, vector<Data::Clouds::State>& StateVector, vector<Data::Clouds::Point3D>& Normals, vector<pair<int, Math::Point3D>>& Candidates, vector<pair<int, Math::Point3D>>& BackupCandidates) {
		if (!Cloud || PointsRangeObj.size() == 0) return;

		std::vector<Data::Clouds::State> ExistingStates;
		PointsRangeObj.GetStates(ExistingStates);
		Data::Clouds::PointsRange SphericalRange;
		vector<Data::Clouds::Point3D> SphericalNeighbourPoints;

		int Idx = 0;

		for (const auto& Point : ogx::Data::Clouds::RangeLocalXYZConst(PointsRangeObj))
		{
			if (ExistingStates[Idx].test(Data::Clouds::PS_DELETED)) {
				StateVector.push_back(ExistingStates[Idx]);
				Normals.push_back(Data::Clouds::Point3D(0.f, 0.f, 1.f));
			}
			else {
				// gather neighbouring points to analyze local surface's orientation
				auto SphericalSearchKernelObj = ogx::Data::Clouds::SphericalSearchKernel(ogx::Math::Sphere3D(NormalRadius, Point.cast<double>()));
				SphericalRange.clear();
				SphericalNeighbourPoints.clear();

				Cloud->GetAccess().FindPoints(SphericalSearchKernelObj, SphericalRange);
				SphericalRange.GetXYZ(SphericalNeighbourPoints);

				// perform only if there are enough neighbors to compute a plane
				if (SphericalNeighbourPoints.size() < 3) {
					StateVector.push_back(Data::Clouds::State());
					auto PtPair = make_pair(Idx, Point.cast<double>());
					Candidates.push_back(PtPair);
					BackupCandidates.push_back(PtPair);
					Normals.push_back(Data::Clouds::Point3D(0.f, 0.f, 1.f));
					Idx++;
					continue;
				}

				// calculate local plane orientation
				auto Plane = ogx::Math::CalcBestPlane3D(SphericalNeighbourPoints.begin(), SphericalNeighbourPoints.end());
				ogx::Math::Point3D Normal = Plane.normal();
				if (Normal.z() < 0) Normal = -Normal;
				
				// remove point if oriented upwards (remove horizontal surfaces)
				if (Normal.z() > NormalThresholdPoints) {
					Data::Clouds::State State;
					State.set(Data::Clouds::PS_DELETED);
					StateVector.push_back(State);
				}
				// else: accept the point
				else {
					StateVector.push_back(Data::Clouds::State());
					auto PtPair = make_pair(Idx, Point.cast<double>());
					Candidates.push_back(PtPair);
					BackupCandidates.push_back(PtPair);
				}
				Normals.push_back(Normal.cast<float>());
			}
			Idx++;
		}
		OGX_LINE.Msg(ogx::Level::Warning, L"Normal filter done, candidates: " + to_wstring(Candidates.size()));
	}

	bool FindPlane(Data::Clouds::ICloud* Cloud, vector<pair<int, Math::Point3D>>& Candidates, int WallId) {
		if (!Cloud || Candidates.empty()) return false;

		// pick a random (point) candidate 
		auto RandomCandidate = Candidates[rand() % Candidates.size()];

		Data::Clouds::PointsRange Neighbours;
		// find neighbours
		auto Kernel = Data::Clouds::SphericalSearchKernel(Math::Sphere3D(PlaneBaseRadius, RandomCandidate.second.cast<double>()));
		Cloud->GetAccess().FindPoints(Kernel, Neighbours);

		vector<Data::Clouds::Point3D> NeighbourXyz;
		Neighbours.GetXYZ(NeighbourXyz);

		// make sure that there are at least 3 neighbours to calculate the plane
		if (NeighbourXyz.size() < static_cast<size_t>(max(3, MinNeighbours))) return false;

		int GuardedRansacIterations = max(1, RansacIterations);

		auto RansacObj = Math::Plane3DRansac(PlaneThickness, GuardedRansacIterations);
		Math::Plane3D Plane;
		vector<Math::Point3D> NeighbourXyzDouble;

		for (const auto& Pt : NeighbourXyz)
			NeighbourXyzDouble.push_back(Pt.cast<double>());

		RansacObj.findPlane3D(NeighbourXyzDouble, Plane);
		Math::Point3D Normal = Plane.normal();

		// advance if plane is vertical enough
		if (abs(Normal.z()) < NormalThresholdWall) {
			BfsRegionGrowing(Candidates, RandomCandidate, Plane, WallId);

			int InlierCount = 0;
			for (auto& Pt : Candidates) {
				if (Visited[Pt.first] && WallLabels[Pt.first] == WallId) InlierCount++;
			}

			if (InlierCount < MinInliersCount) {
				for (auto& Pt : Candidates) {
					if (WallLabels[Pt.first] == WallId) {
						WallLabels[Pt.first] = 0;
						Visited[Pt.first] = false;
					}
				}
				return false;
			}

			WallDimensions Dimensions;
			Dimensions.CalculateWallDims(Candidates, WallLabels, Visited, WallId);

			// ignore small planes
			if (Dimensions.Height < MinWallHeight || Dimensions.Width < MinWallWidth) {
				for (auto& Pt : Candidates) {
					if (WallLabels[Pt.first] == WallId) {
						WallLabels[Pt.first] = 0;
						Visited[Pt.first] = false;
					}
				}
				return false;
			}

			// update remaining candidates
			vector<pair<int, Math::Point3D>> Remaining;
			for (auto& Pt : Candidates) {
				if (!Visited[Pt.first]) Remaining.push_back(Pt);
			}
			Candidates = Remaining;

			OGX_LINE.Msg(ogx::Level::Warning, L"Wall found: " + to_wstring(WallId) + L" inliers: " + to_wstring(InlierCount) + L" | H=" + to_wstring(Dimensions.Height) + L"m, W=" + to_wstring(Dimensions.Width) + L"m");
			return true;
		}
		return false;
	}

	int RunMultiScaleRansac(Data::Clouds::ICloud* Cloud, vector<pair<int, Math::Point3D>>& Candidates) {
		if (!Cloud || Candidates.empty()) return 0;

		int WallId = 1;
		double BaseRadius = PlaneBaseRadius;
		int BaseMinInliers = MinInliersCount;

		int GuardedScalesCount = max(1, ScalesCount);

		// iterate the scale
		for (int Scale = 0; Scale < GuardedScalesCount; ++Scale)
		{	
			// reduce search radius & minInliers tresholdby half at each scale level
			PlaneBaseRadius = BaseRadius / pow(2.0, Scale);
			MinInliersCount = static_cast<int>(BaseMinInliers / pow(2.0, Scale));

			if (MinInliersCount < MinNeighbours) MinInliersCount = MinNeighbours;

			OGX_LINE.Msg(ogx::Level::Info, L"Scale layer " + to_wstring(Scale + 1) + L"/" + to_wstring(GuardedScalesCount) +
				L" | Radius: " + to_wstring(PlaneBaseRadius) + L"m | Min Inliers: " + to_wstring(MinInliersCount));

			int FailedAttempts = 0;
			
			while (FailedAttempts < RansacIterations && !Candidates.empty())
			{
				if (FindPlane(Cloud, Candidates, WallId)) {
					WallId++;
					FailedAttempts = 0;
				}
				else {
					FailedAttempts++;
				}
			}
		}

		// restore parameter states
		PlaneBaseRadius = BaseRadius;
		MinInliersCount = BaseMinInliers;
		return WallId - 1;
	}

	// function for additional BFS at the end
	void EndBFS(int TotalPlanesFound, const vector<pair<int, Math::Point3D>>& BackupCandidates) {
		if (!RunDetailBfs || TotalPlanesFound <= 0 || BackupCandidates.empty()) return;

		OGX_LINE.Msg(ogx::Level::Info, L"Starting spatial BFS detail absorption phase...");
		int AbsorbedCount = 0;
		
		// iterate through all walls
		for (int WallIdIdx = 1; WallIdIdx <= TotalPlanesFound; ++WallIdIdx)
		{
			// seed queue with all verified points belonging to the currently iterated-on wall
			queue<pair<int, Math::Point3D>> DetailQueue;
			for (const auto& Pt : BackupCandidates) {
				if (Visited[Pt.first] && static_cast<int>(WallLabels[Pt.first]) == WallIdIdx) {
					DetailQueue.push(Pt);
				}
			}

			while (!DetailQueue.empty())
			{
				auto ActivePoint = DetailQueue.front();
				DetailQueue.pop();

				for (const auto& Target : BackupCandidates)
				{
					if (!Visited[Target.first]) {
						double SpatialDist = (Target.second - ActivePoint.second).norm();
						if (SpatialDist < DetailBfsRadius) {
							Visited[Target.first] = true;
							WallLabels[Target.first] = static_cast<float>(WallIdIdx);
							DetailQueue.push(Target);
							AbsorbedCount++;
						}
					}
				}
			}
		}
		OGX_LINE.Msg(ogx::Level::Warning, L"Detail absorption finished! Absorbed " + to_wstring(AbsorbedCount) + L" structural elements.");
	}

	// function for labels, colors, and deletion flags
	void FinalizeCloudLayers(Data::Clouds::ICloud* Cloud, Data::Clouds::PointsRange& PointsRangeObj, vector<Data::Clouds::State>& StateVector, vector<Data::Clouds::Point3D>& Normals) {
		if (!Cloud || PointsRangeObj.size() == 0) return;

		auto Layers = Cloud->FindLayers(L"WallID");
		auto Layer = Layers.empty() ? Cloud->CreateLayer(L"WallID", 0.f) : Layers[0];

		static const Data::Clouds::Color WallColors[] = {
			{ 255,   0,   0, 255 }, {   0, 255,   0, 255 }, {   0,   0, 255, 255 },
			{ 255, 255,   0, 255 }, {   0, 255, 255, 255 }, { 255,   0, 255, 255 },
			{ 255, 128,   0, 255 }, { 128, 255,   0, 255 }, {   0, 255, 128, 255 },
			{   0, 128, 255, 255 }, { 128,   0, 255, 255 }, { 255,   0, 128, 255 },
			{   0, 128, 128, 255 }, { 128,   0, 128, 255 },
		};
		const int NumWallColors = (int)(sizeof(WallColors) / sizeof(WallColors[0]));

		std::vector<Data::Clouds::Color> Colors;
		Colors.reserve(PointsRangeObj.size());

		// assign colours to points in the wall
		for (size_t I = 0; I < PointsRangeObj.size(); I++) {
			int Id = (int)WallLabels[I];
			if (Id == 0) {
				// paint gray if ignored
				Colors.push_back(Data::Clouds::Color(128, 128, 128, 255));
			}
			else {
				Colors.push_back(WallColors[(Id - 1) % NumWallColors]);
			}
		}
		PointsRangeObj.SetColors(Colors);

		for (size_t I = 0; I < StateVector.size(); I++) {
			if (WallLabels[I] == 0.f) {
				StateVector[I].set(Data::Clouds::PS_DELETED);
			}
		}

		PointsRangeObj.SetLayerVals(WallLabels, *Layer);
		PointsRangeObj.SetNormals(Normals);
		PointsRangeObj.SetStates(StateVector);
		Cloud->GetAccess().DeletePoints();
	}

	// Main method combining the rest into one pipeline
	virtual void Run(Context& ContextObj)
	{
		srand(time(0));

		auto Node = ContextObj.m_project->TransTreeFindNode(NodeId);
		auto NewNode = DuplicateNode(ContextObj.Project(), Node, Node->GetParent());
		auto Cloud = GetCloud(ContextObj, NewNode);

		if (!Cloud) {
			ReportError(L"Retrieved cloud node pointer is Null.");
			return;
		}

		Data::Clouds::PointsRange PointsRangeObj;
		Cloud->GetAccess().GetAllPoints(PointsRangeObj);
		OGX_LINE.Msg(ogx::Level::Warning, L"Total points in cloud: " + to_wstring(PointsRangeObj.size()));

		if (PointsRangeObj.size() == 0) {
			ReportError(L"Input point cloud is empty.");
			return;
		}

		UnderSample(PointsRangeObj);
		Cloud->GetAccess().DeletePoints();
		PointsRangeObj.clear();
		Cloud->GetAccess().GetAllPoints(PointsRangeObj);

		FilterIsolatedPoints(Cloud, PointsRangeObj);
		PointsRangeObj.clear();
		Cloud->GetAccess().GetAllPoints(PointsRangeObj);

		WallLabels.assign(PointsRangeObj.size(), 0.f);
		Visited.assign(PointsRangeObj.size(), false);

		vector<Data::Clouds::State> StateVector;
		StateVector.reserve(PointsRangeObj.size());
		vector<Data::Clouds::Point3D> Normals;
		Normals.reserve(PointsRangeObj.size());

		vector<pair<int, Math::Point3D>> Candidates;
		vector<pair<int, Math::Point3D>> BackupCandidates;

		NormalFilter(Cloud, PointsRangeObj, StateVector, Normals, Candidates, BackupCandidates);

		int TotalPlanesFound = RunMultiScaleRansac(Cloud, Candidates);
		OGX_LINE.Msg(ogx::Level::Warning, L"Multi-scale RANSAC completed. Total walls found: " + to_wstring(TotalPlanesFound));

		EndBFS(TotalPlanesFound, BackupCandidates);
		FinalizeCloudLayers(Cloud, PointsRangeObj, StateVector, Normals);
	}
};
OGX_EXPORT_METHOD(FinalFilter)
