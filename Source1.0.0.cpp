
#include <vector>
#include <algorithm>
using namespace std;

class threeD_test_Point {
public:
	float x;
	float y;
	float z;

public:
	void setXYZ(float xx, float yy, float zz) {
		x = xx;
		y = yy;
		z = zz;
	}

};

struct threeD_test_VoxelData {
	unsigned short  VoxelIndex;
	threeD_test_Point Voxel_center;
	vector<unsigned short> Members;
};
/*
		BVG
		Abbreviation for  BabeVoxelGrid
*/
class threeD_test_BVG {
public:
	int minX;
	int minY;
	int minZ;
	int maxX;
	int maxY;
	int maxZ;
	float VoxelOrigin_X;
	float VoxelOrigin_Y;
	float VoxelOrigin_Z;
	int x_axis_len;
	int y_axis_len;
	int z_axis_len;
	int xy_slice;
	unsigned short voxelTotal;
	/* Resolution: 1cm */
	float VoxelSize;
	float VoxelRadius;
	unsigned short PointCloudSize;
	vector<threeD_test_VoxelData> Essence;
	vector<unsigned short > Edge_Face;


public:
	threeD_test_BVG(threeD_test_Point *pc, unsigned short point_size, float voxelsize = 10) {
		PointCloudSize = point_size;
		VoxelSize = voxelsize;
		VoxelRadius = VoxelSize / 2;
		float maxx = -FLT_MAX;
		float maxy = -FLT_MAX;
		float maxz = -FLT_MAX;
		float minx = FLT_MAX;
		float miny = FLT_MAX;
		float minz = FLT_MAX;
		for (int i = 0; i != point_size; i++) {
			if (pc[i].x > maxx && fabs(pc[i].x - maxx) > FloatMinErrorValue) maxx = pc[i].x;
			if (pc[i].y > maxy && fabs(pc[i].y - maxy) > FloatMinErrorValue) maxy = pc[i].y;
			if (pc[i].z > maxz && fabs(pc[i].z - maxz) > FloatMinErrorValue) maxz = pc[i].z;
			if (pc[i].x < minx && fabs(pc[i].x - minx) > FloatMinErrorValue) minx = pc[i].x;
			if (pc[i].y < miny && fabs(pc[i].y - miny) > FloatMinErrorValue) miny = pc[i].y;
			if (pc[i].z < minz&& fabs(pc[i].z - minz) > FloatMinErrorValue) minz = pc[i].z;
		}

		minX = 0;
		minY = 0;
		minZ = 0;

		int count_l = 0;
		int count_r = 0;
		//X
		VoxelOrigin_X = floorf(maxx + minx) / 2;
		while ((VoxelOrigin_X - (count_l + 0.5)*VoxelSize) > minx && fabs((VoxelOrigin_X - (count_l + 0.5)*VoxelSize) - minx) > FloatMinErrorValue) {
			count_l++;
		}
		while ((VoxelOrigin_X + (count_r + 0.5)*VoxelSize) < maxx && fabs((VoxelOrigin_X + (count_r + 0.5)*VoxelSize) - maxx) > FloatMinErrorValue) {
			count_r++;
		}
		maxX = count_l + count_r + 2;
		VoxelOrigin_X -= (count_l + 0.5) * VoxelSize;
		count_l = 0;
		count_r = 0;

		//Y
		VoxelOrigin_Y = floorf(maxy + miny) / 1;
		while ((VoxelOrigin_Y - (count_l + 0.5)*VoxelSize) > miny && fabs((VoxelOrigin_Y - (count_l + 0.5)*VoxelSize) - miny) > FloatMinErrorValue) {
			count_l++;
		}
		while ((VoxelOrigin_Y + (count_r + 0.5)*VoxelSize) < maxy&& fabs((VoxelOrigin_Y + (count_r + 0.5)*VoxelSize) - maxy) > FloatMinErrorValue) {
			count_r++;
		}
		maxY = count_l + count_r + 2;
		VoxelOrigin_Y -= (count_l + 0.5) * VoxelSize;
		count_l = 0;
		count_r = 0;

		//Z
		VoxelOrigin_Z = floorf(maxz + minz) / 2;
		while ((VoxelOrigin_Z - (count_l + 0.5)*VoxelSize) > minz && fabs((VoxelOrigin_Z - (count_l + 0.5)*VoxelSize) - minz) > FloatMinErrorValue) {
			count_l++;
		}
		while ((VoxelOrigin_Z + (count_r + 0.5)*VoxelSize) < maxz && fabs((VoxelOrigin_Z + (count_r + 0.5)*VoxelSize) - maxz) > FloatMinErrorValue) {
			count_r++;
		}
		maxZ = count_l + count_r + 2;
		VoxelOrigin_Z -= (count_l + 0.5) * VoxelSize;
		count_l = 0;
		count_r = 0;

		//cout << VoxelOrigin_X << endl;
		//cout << VoxelOrigin_Y << endl;
		//cout << VoxelOrigin_Z << endl;


		x_axis_len = (maxX - minX);
		y_axis_len = (maxY - minY);
		z_axis_len = (maxZ - minZ);
		xy_slice = x_axis_len * y_axis_len;
		voxelTotal = x_axis_len * y_axis_len * z_axis_len;




	}

	unsigned char IsInsidePoint(threeD_test_Point p, int voxelindex) {
		threeD_test_Point vc = getVoxelCenter(voxelindex);
		if (abs(vc.x - p.x) > VoxelRadius) return 0x00;
		if (abs(vc.y - p.y) > VoxelRadius) return 0x00;
		if (abs(vc.z - p.z) > VoxelRadius) return 0x00;
		return 0x01;
	}

	unsigned char IsInsidePoint(threeD_test_Point p, threeD_test_Point VoxelCenterPoint) {
		//threeD_test_Point vc = getVoxelCenter(voxelindex);
		if (abs(VoxelCenterPoint.x - p.x) > VoxelRadius) return 0x00;
		if (abs(VoxelCenterPoint.y - p.y) > VoxelRadius) return 0x00;
		if (abs(VoxelCenterPoint.z - p.z) > VoxelRadius) return 0x00;
		return 0x01;
	}

	void init(threeD_test_Point *pc) {
		threeD_test_Point last_VoxelCenterPoint;
		int temp_voxelindex = 0;
		temp_voxelindex = calcVoxelIndex(pc[0]);
		last_VoxelCenterPoint = getVoxelCenter(temp_voxelindex);
		int vdindex = getVoxelData(temp_voxelindex);
		int vdindex_last = vdindex;
		Essence.at(vdindex).Voxel_center = last_VoxelCenterPoint;
		Essence.at(vdindex).Members.push_back(0);
		for (int i = 1; i != PointCloudSize; i++) {
			if (IsInsidePoint(pc[i], last_VoxelCenterPoint)) {
				Essence.at(vdindex_last).Members.push_back(i);
			}
			else {
				temp_voxelindex = calcVoxelIndex(pc[i]);
				last_VoxelCenterPoint = getVoxelCenter(temp_voxelindex);
				vdindex = getVoxelData(temp_voxelindex);
				vdindex_last = vdindex;
				Essence.at(vdindex_last).Voxel_center = last_VoxelCenterPoint;
				Essence.at(vdindex_last).Members.push_back(i);
			}
		}


	}

	void setExternalRange(int minx, int miny, int minz, int maxx, int maxy, int maxz) {
		minX = minx;
		minY = miny;
		minZ = minz;
		maxX = maxx;
		maxY = maxy;
		maxZ = maxz;
	}

	unsigned short calcVoxelIndex(threeD_test_Point p) {
		float x = p.x;
		float y = p.y;
		float z = p.z;

		int xvalue;
		if ((x - VoxelOrigin_X) / VoxelSize - floorf((x - VoxelOrigin_X) / VoxelSize) <= VoxelRadius) {
			xvalue = (int)floorf((x - VoxelOrigin_X) / VoxelSize);
		}
		else {
			xvalue = (int)ceilf((x - VoxelOrigin_X) / VoxelSize);
		}

		int yvalue;
		if ((y - VoxelOrigin_Y) / VoxelSize - floorf((y - VoxelOrigin_Y) / VoxelSize) <= VoxelRadius) {
			yvalue = (int)floorf((y - VoxelOrigin_Y) / VoxelSize);
		}
		else {
			yvalue = (int)ceilf((y - VoxelOrigin_Y) / VoxelSize);
		}

		int zvalue;
		if ((z - VoxelOrigin_Z) / VoxelSize - floorf((z - VoxelOrigin_Y) / VoxelSize) <= VoxelRadius) {
			zvalue = (int)floorf((z - VoxelOrigin_Z) / VoxelSize);
		}
		else {
			zvalue = (int)ceilf((z - VoxelOrigin_Z) / VoxelSize);
		}

		int result = 0;
		result += zvalue * (xy_slice);
		result += yvalue * (x_axis_len);
		result += xvalue;

		return result;

	}

	int getVoxelData(int voxelindex) {
		for (int i = 0; i != Essence.size(); i++) {
			if (Essence.at(i).VoxelIndex == voxelindex) return i;
		}
		threeD_test_VoxelData vd;
		vd.VoxelIndex = voxelindex;
		vd.Members.clear();
		vd.Voxel_center = getVoxelCenter(voxelindex);
		Essence.push_back(vd);
		return Essence.size() - 1;
	}

	vector<unsigned short> getVoxelPointIndices(int voxelindex) {
		for (int i = 0; i != Essence.size(); i++) {
			if (Essence.at(i).VoxelIndex == voxelindex) return Essence.at(i).Members;
		}
		vector<unsigned short> nullv;
		nullv.clear();
		return nullv;
	}

	threeD_test_Point getVoxelCenter(int voxelindex) {
		int zvalue = (int)floor((float)voxelindex / xy_slice);
		voxelindex -= zvalue * xy_slice;
		int yvalue = (int)floor((float)voxelindex / x_axis_len);
		voxelindex -= yvalue * x_axis_len;
		int xvalue = voxelindex;

		float x = (xvalue + 0.5) *VoxelSize + VoxelOrigin_X;
		float y = (yvalue + 0.5) *VoxelSize + VoxelOrigin_Y;
		float z = (zvalue + 0.5) *VoxelSize + VoxelOrigin_Z;
		threeD_test_Point p;
		p.x = x;
		p.y = y;
		p.z = z;
		return p;
	}


	unsigned char IsValidVoxel(threeD_test_Point *pc, int voxelindex) {
		int vdindex = getVoxelData(voxelindex);
		threeD_test_VoxelData vd = Essence.at(vdindex);
		if (vd.Members.size() != 0) {
			return 0x01;
		}
		else {
			return 0x00;
		}
	}

	unsigned char IsValidVoxel(int voxelindex) {
		int vdindex = getVoxelData(voxelindex);
		threeD_test_VoxelData vd = Essence.at(vdindex);
		if (vd.Members.size() > 0) {
			return 0x01;
		}
		else {
			return 0x00;
		}
	}

	threeD_test_Point getVoxelCenterBy_voxelindex(int voxelindex) {
		int vdindex = getVoxelData(voxelindex);
		threeD_test_VoxelData vd = Essence.at(vdindex);
		return vd.Voxel_center;
	}

	vector<unsigned short > getExteriorVoxelIndices() {

		float maxx = -FLT_MAX;
		float maxy = -FLT_MAX;
		float maxz = -FLT_MAX;
		float minx = FLT_MAX;
		float miny = FLT_MAX;
		float minz = FLT_MAX;


		vector<unsigned short> maxx_vdindices;
		vector<unsigned short> maxy_vdindices;
		vector<unsigned short> maxz_vdindices;
		vector<unsigned short> minx_vdindices;
		vector<unsigned short> miny_vdindices;
		vector<unsigned short> minz_vdindices;

		for (int i = 0; i != Essence.size(); i++) {
			if (IsValidVoxel(Essence.at(i).VoxelIndex)) {
				if (Essence.at(i).Voxel_center.x > maxx  && abs(Essence.at(i).Voxel_center.x - maxx) > FloatMinErrorValue) {
					maxx = Essence.at(i).Voxel_center.x;
					maxx_vdindices.clear();
					maxx_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (Essence.at(i).Voxel_center.y > maxy  && abs(Essence.at(i).Voxel_center.y - maxy) > FloatMinErrorValue) {
					maxy = Essence.at(i).Voxel_center.y;
					maxy_vdindices.clear();
					maxy_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (Essence.at(i).Voxel_center.z > maxz  && abs(Essence.at(i).Voxel_center.z - maxz) > FloatMinErrorValue) {
					maxz = Essence.at(i).Voxel_center.z;
					maxz_vdindices.clear();
					maxz_vdindices.push_back(Essence.at(i).VoxelIndex);
				}



				if (Essence.at(i).Voxel_center.x < minx  && abs(Essence.at(i).Voxel_center.x - minx) > FloatMinErrorValue) {
					minx = Essence.at(i).Voxel_center.x;
					minx_vdindices.clear();
					minx_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (Essence.at(i).Voxel_center.y < miny  && abs(Essence.at(i).Voxel_center.y - miny) > FloatMinErrorValue) {
					miny = Essence.at(i).Voxel_center.y;
					miny_vdindices.clear();
					miny_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (Essence.at(i).Voxel_center.z < minz  && abs(Essence.at(i).Voxel_center.z - minz) > FloatMinErrorValue) {
					minz = Essence.at(i).Voxel_center.z;
					minz_vdindices.clear();
					minz_vdindices.push_back(Essence.at(i).VoxelIndex);
				}


				if (abs(Essence.at(i).Voxel_center.x - maxx) < FloatMinErrorValue) {
					maxx_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (abs(Essence.at(i).Voxel_center.y - maxy) < FloatMinErrorValue) {
					maxy_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (abs(Essence.at(i).Voxel_center.z - maxz) < FloatMinErrorValue) {
					maxz_vdindices.push_back(Essence.at(i).VoxelIndex);
				}





				if (abs(Essence.at(i).Voxel_center.x - minx) < FloatMinErrorValue) {
					minx_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (abs(Essence.at(i).Voxel_center.y - miny) < FloatMinErrorValue) {
					miny_vdindices.push_back(Essence.at(i).VoxelIndex);
				}
				if (abs(Essence.at(i).Voxel_center.z - minz) < FloatMinErrorValue) {
					minz_vdindices.push_back(Essence.at(i).VoxelIndex);
				}

			}

		}

		vector<unsigned short> edge_face;
		edge_face.clear();

		edge_face.insert(edge_face.end(), maxx_vdindices.begin(), maxx_vdindices.end());
		edge_face.insert(edge_face.end(), maxy_vdindices.begin(), maxy_vdindices.end());
		edge_face.insert(edge_face.end(), maxz_vdindices.begin(), maxz_vdindices.end());
		edge_face.insert(edge_face.end(), minx_vdindices.begin(), minx_vdindices.end());
		edge_face.insert(edge_face.end(), miny_vdindices.begin(), miny_vdindices.end());
		edge_face.insert(edge_face.end(), minz_vdindices.begin(), minz_vdindices.end());

		sort(edge_face.begin(), edge_face.end());
		edge_face.erase(unique(edge_face.begin(), edge_face.end()), edge_face.end());

		return edge_face;
	}

	void initEdgeFace() {
		Edge_Face = getExteriorVoxelIndices();
	}

	unsigned char IsEdgeVoxel(unsigned short voxelindex) {
		for (int i = 0; i != Edge_Face.size(); i++) {
			if (Edge_Face.at(i) == voxelindex) return 0x01;
		}
		return 0x00;
	}

};
