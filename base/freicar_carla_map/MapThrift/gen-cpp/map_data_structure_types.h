/**
 * Autogenerated by Thrift Compiler (0.12.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
#ifndef map_data_structure_TYPES_H
#define map_data_structure_TYPES_H

#include <thrift/TApplicationException.h>
#include <thrift/TBase.h>
#include <thrift/Thrift.h>
#include <thrift/protocol/TProtocol.h>
#include <thrift/stdcxx.h>
#include <thrift/transport/TTransport.h>

#include <iosfwd>

namespace map_thrift {

struct MessageOp {
    enum type { ADD = 1, UPDATE_WHOLE = 2, UPDATE_PART = 3, DELETE = 4, POSE_UPDATE = 5, POSE_DELETE = 6 };
};

extern const std::map<int, const char*> _MessageOp_VALUES_TO_NAMES;

std::ostream& operator<<(std::ostream& out, const MessageOp::type& val);

struct LaneMarkingType {
    enum type { SOLID = 1, DASHED = 2, CENTER_SOLID = 3, CENTER_DASHED = 4 };
};

extern const std::map<int, const char*> _LaneMarkingType_VALUES_TO_NAMES;

std::ostream& operator<<(std::ostream& out, const LaneMarkingType::type& val);

struct LaneType {
    enum type { NORMAL = 1, CAR_LANE = 2, PEDESTRIAN_LANE = 3 };
};

extern const std::map<int, const char*> _LaneType_VALUES_TO_NAMES;

std::ostream& operator<<(std::ostream& out, const LaneType::type& val);

struct LaneDirection {
    enum type { RIGHT = 1, LEFT = 2 };
};

extern const std::map<int, const char*> _LaneDirection_VALUES_TO_NAMES;

std::ostream& operator<<(std::ostream& out, const LaneDirection::type& val);

struct LaneObjectType {
    enum type { STOPLINE = 1, CROSSWALK = 2, PARKING = 3, SIGN = 4 };
};

extern const std::map<int, const char*> _LaneObjectType_VALUES_TO_NAMES;

std::ostream& operator<<(std::ostream& out, const LaneObjectType::type& val);

typedef int64_t int64;

class Uuid;

class Point3D;

class Pose;

class Pivot;

class MapContainer;

class LaneObjectList;

class MapPart;

class LaneMarking;

class LaneMarkingContainer;

class Lane;

class LaneGroup;

class LaneObjectBase;

class Stopline;

class Crosswalk;

class Parking;

class Roadsign;

class PointViz;

class MapMessage;

class CarMessage;

class ThriftImage;

class Uuid : public virtual ::apache::thrift::TBase {
   public:
    Uuid(const Uuid&);
    Uuid& operator=(const Uuid&);
    Uuid() : uuid() {}

    virtual ~Uuid() throw();
    std::string uuid;

    void __set_uuid(const std::string& val);

    bool operator==(const Uuid& rhs) const {
        if (!(uuid == rhs.uuid)) return false;
        return true;
    }
    bool operator!=(const Uuid& rhs) const { return !(*this == rhs); }

    bool operator<(const Uuid&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Uuid& a, Uuid& b);

std::ostream& operator<<(std::ostream& out, const Uuid& obj);

class Point3D : public virtual ::apache::thrift::TBase {
   public:
    Point3D(const Point3D&);
    Point3D& operator=(const Point3D&);
    Point3D() : x(0), y(0), z(static_cast<double>(0)) {}

    virtual ~Point3D() throw();
    double x;
    double y;
    double z;

    void __set_x(const double val);

    void __set_y(const double val);

    void __set_z(const double val);

    bool operator==(const Point3D& rhs) const {
        if (!(x == rhs.x)) return false;
        if (!(y == rhs.y)) return false;
        if (!(z == rhs.z)) return false;
        return true;
    }
    bool operator!=(const Point3D& rhs) const { return !(*this == rhs); }

    bool operator<(const Point3D&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Point3D& a, Point3D& b);

std::ostream& operator<<(std::ostream& out, const Point3D& obj);

class Pose : public virtual ::apache::thrift::TBase {
   public:
    Pose(const Pose&);
    Pose& operator=(const Pose&);
    Pose() : orientation(0) {}

    virtual ~Pose() throw();
    Point3D position;
    double orientation;

    void __set_position(const Point3D& val);

    void __set_orientation(const double val);

    bool operator==(const Pose& rhs) const {
        if (!(position == rhs.position)) return false;
        if (!(orientation == rhs.orientation)) return false;
        return true;
    }
    bool operator!=(const Pose& rhs) const { return !(*this == rhs); }

    bool operator<(const Pose&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Pose& a, Pose& b);

std::ostream& operator<<(std::ostream& out, const Pose& obj);

typedef struct _Pivot__isset {
    _Pivot__isset() : id(false), pose(false), is_valid(true) {}
    bool id : 1;
    bool pose : 1;
    bool is_valid : 1;
} _Pivot__isset;

class Pivot : public virtual ::apache::thrift::TBase {
   public:
    Pivot(const Pivot&);
    Pivot& operator=(const Pivot&);
    Pivot() : is_valid(false) {}

    virtual ~Pivot() throw();
    Uuid id;
    Pose pose;
    bool is_valid;

    _Pivot__isset __isset;

    void __set_id(const Uuid& val);

    void __set_pose(const Pose& val);

    void __set_is_valid(const bool val);

    bool operator==(const Pivot& rhs) const {
        if (!(id == rhs.id)) return false;
        if (!(pose == rhs.pose)) return false;
        if (!(is_valid == rhs.is_valid)) return false;
        return true;
    }
    bool operator!=(const Pivot& rhs) const { return !(*this == rhs); }

    bool operator<(const Pivot&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Pivot& a, Pivot& b);

std::ostream& operator<<(std::ostream& out, const Pivot& obj);

typedef struct _MapContainer__isset {
    _MapContainer__isset() : map_parts(false), pivot(false) {}
    bool map_parts : 1;
    bool pivot : 1;
} _MapContainer__isset;

class MapContainer : public virtual ::apache::thrift::TBase {
   public:
    MapContainer(const MapContainer&);
    MapContainer& operator=(const MapContainer&);
    MapContainer() {}

    virtual ~MapContainer() throw();
    std::vector<MapPart> map_parts;
    Pivot pivot;

    _MapContainer__isset __isset;

    void __set_map_parts(const std::vector<MapPart>& val);

    void __set_pivot(const Pivot& val);

    bool operator==(const MapContainer& rhs) const {
        if (!(map_parts == rhs.map_parts)) return false;
        if (!(pivot == rhs.pivot)) return false;
        return true;
    }
    bool operator!=(const MapContainer& rhs) const { return !(*this == rhs); }

    bool operator<(const MapContainer&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(MapContainer& a, MapContainer& b);

std::ostream& operator<<(std::ostream& out, const MapContainer& obj);

typedef struct _LaneObjectList__isset {
    _LaneObjectList__isset() : stoplines(false), crosswalks(false), parking_lots(false), roadsigns(false) {}
    bool stoplines : 1;
    bool crosswalks : 1;
    bool parking_lots : 1;
    bool roadsigns : 1;
} _LaneObjectList__isset;

class LaneObjectList : public virtual ::apache::thrift::TBase {
   public:
    LaneObjectList(const LaneObjectList&);
    LaneObjectList& operator=(const LaneObjectList&);
    LaneObjectList() {}

    virtual ~LaneObjectList() throw();
    std::vector<Stopline> stoplines;
    std::vector<Crosswalk> crosswalks;
    std::vector<Parking> parking_lots;
    std::vector<Roadsign> roadsigns;

    _LaneObjectList__isset __isset;

    void __set_stoplines(const std::vector<Stopline>& val);

    void __set_crosswalks(const std::vector<Crosswalk>& val);

    void __set_parking_lots(const std::vector<Parking>& val);

    void __set_roadsigns(const std::vector<Roadsign>& val);

    bool operator==(const LaneObjectList& rhs) const {
        if (!(stoplines == rhs.stoplines)) return false;
        if (!(crosswalks == rhs.crosswalks)) return false;
        if (!(parking_lots == rhs.parking_lots)) return false;
        if (!(roadsigns == rhs.roadsigns)) return false;
        return true;
    }
    bool operator!=(const LaneObjectList& rhs) const { return !(*this == rhs); }

    bool operator<(const LaneObjectList&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(LaneObjectList& a, LaneObjectList& b);

std::ostream& operator<<(std::ostream& out, const LaneObjectList& obj);

typedef struct _MapPart__isset {
    _MapPart__isset() : lanes(false), lane_groups(false), lane_markings(false), lane_objects(false) {}
    bool lanes : 1;
    bool lane_groups : 1;
    bool lane_markings : 1;
    bool lane_objects : 1;
} _MapPart__isset;

class MapPart : public virtual ::apache::thrift::TBase {
   public:
    MapPart(const MapPart&);
    MapPart& operator=(const MapPart&);
    MapPart() {}

    virtual ~MapPart() throw();
    std::vector<Lane> lanes;
    std::vector<LaneGroup> lane_groups;
    std::vector<LaneMarking> lane_markings;
    LaneObjectList lane_objects;

    _MapPart__isset __isset;

    void __set_lanes(const std::vector<Lane>& val);

    void __set_lane_groups(const std::vector<LaneGroup>& val);

    void __set_lane_markings(const std::vector<LaneMarking>& val);

    void __set_lane_objects(const LaneObjectList& val);

    bool operator==(const MapPart& rhs) const {
        if (!(lanes == rhs.lanes)) return false;
        if (!(lane_groups == rhs.lane_groups)) return false;
        if (!(lane_markings == rhs.lane_markings)) return false;
        if (!(lane_objects == rhs.lane_objects)) return false;
        return true;
    }
    bool operator!=(const MapPart& rhs) const { return !(*this == rhs); }

    bool operator<(const MapPart&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(MapPart& a, MapPart& b);

std::ostream& operator<<(std::ostream& out, const MapPart& obj);

typedef struct _LaneMarking__isset {
    _LaneMarking__isset() : type(false), points(false), visibility(false) {}
    bool type : 1;
    bool points : 1;
    bool visibility : 1;
} _LaneMarking__isset;

class LaneMarking : public virtual ::apache::thrift::TBase {
   public:
    LaneMarking(const LaneMarking&);
    LaneMarking& operator=(const LaneMarking&);
    LaneMarking() : type((LaneMarkingType::type)0), visibility(0) {}

    virtual ~LaneMarking() throw();
    LaneMarkingType::type type;
    std::vector<Point3D> points;
    Uuid id;
    bool visibility;

    _LaneMarking__isset __isset;

    void __set_type(const LaneMarkingType::type val);

    void __set_points(const std::vector<Point3D>& val);

    void __set_id(const Uuid& val);

    void __set_visibility(const bool val);

    bool operator==(const LaneMarking& rhs) const {
        if (!(type == rhs.type)) return false;
        if (!(points == rhs.points)) return false;
        if (!(id == rhs.id)) return false;
        if (!(visibility == rhs.visibility)) return false;
        return true;
    }
    bool operator!=(const LaneMarking& rhs) const { return !(*this == rhs); }

    bool operator<(const LaneMarking&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(LaneMarking& a, LaneMarking& b);

std::ostream& operator<<(std::ostream& out, const LaneMarking& obj);

typedef struct _LaneMarkingContainer__isset {
    _LaneMarkingContainer__isset() : left(false), right(false) {}
    bool left : 1;
    bool right : 1;
} _LaneMarkingContainer__isset;

class LaneMarkingContainer : public virtual ::apache::thrift::TBase {
   public:
    LaneMarkingContainer(const LaneMarkingContainer&);
    LaneMarkingContainer& operator=(const LaneMarkingContainer&);
    LaneMarkingContainer() {}

    virtual ~LaneMarkingContainer() throw();
    std::vector<Uuid> left;
    std::vector<Uuid> right;

    _LaneMarkingContainer__isset __isset;

    void __set_left(const std::vector<Uuid>& val);

    void __set_right(const std::vector<Uuid>& val);

    bool operator==(const LaneMarkingContainer& rhs) const {
        if (!(left == rhs.left)) return false;
        if (!(right == rhs.right)) return false;
        return true;
    }
    bool operator!=(const LaneMarkingContainer& rhs) const { return !(*this == rhs); }

    bool operator<(const LaneMarkingContainer&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(LaneMarkingContainer& a, LaneMarkingContainer& b);

std::ostream& operator<<(std::ostream& out, const LaneMarkingContainer& obj);

typedef struct _Lane__isset {
    _Lane__isset()
        : lane_markings(false),
          dir(false),
          outgoing_connections(false),
          incoming_connections(false),
          points(false),
          width(false),
          handle_point(false),
          visibility(false),
          height(false),
          junction_id(false) {}
    bool lane_markings : 1;
    bool dir : 1;
    bool outgoing_connections : 1;
    bool incoming_connections : 1;
    bool points : 1;
    bool width : 1;
    bool handle_point : 1;
    bool visibility : 1;
    bool height : 1;
    bool junction_id : 1;
} _Lane__isset;

class Lane : public virtual ::apache::thrift::TBase {
   public:
    Lane(const Lane&);
    Lane& operator=(const Lane&);
    Lane() : dir((LaneDirection::type)0), type((LaneType::type)0), width(0), visibility(0), height(0), junction_id(0) {}

    virtual ~Lane() throw();
    LaneMarkingContainer lane_markings;
    LaneDirection::type dir;
    std::vector<Uuid> outgoing_connections;
    std::vector<Uuid> incoming_connections;
    LaneType::type type;
    std::vector<Point3D> points;
    double width;
    Uuid id;
    Point3D handle_point;
    bool visibility;
    int64 height;
    int64 junction_id;

    _Lane__isset __isset;

    void __set_lane_markings(const LaneMarkingContainer& val);

    void __set_dir(const LaneDirection::type val);

    void __set_outgoing_connections(const std::vector<Uuid>& val);

    void __set_incoming_connections(const std::vector<Uuid>& val);

    void __set_type(const LaneType::type val);

    void __set_points(const std::vector<Point3D>& val);

    void __set_width(const double val);

    void __set_id(const Uuid& val);

    void __set_handle_point(const Point3D& val);

    void __set_visibility(const bool val);

    void __set_height(const int64 val);

    void __set_junction_id(const int64 val);

    bool operator==(const Lane& rhs) const {
        if (!(lane_markings == rhs.lane_markings)) return false;
        if (!(dir == rhs.dir)) return false;
        if (!(outgoing_connections == rhs.outgoing_connections)) return false;
        if (!(incoming_connections == rhs.incoming_connections)) return false;
        if (!(type == rhs.type)) return false;
        if (!(points == rhs.points)) return false;
        if (!(width == rhs.width)) return false;
        if (!(id == rhs.id)) return false;
        if (!(handle_point == rhs.handle_point)) return false;
        if (!(visibility == rhs.visibility)) return false;
        if (!(height == rhs.height)) return false;
        if (!(junction_id == rhs.junction_id)) return false;
        return true;
    }
    bool operator!=(const Lane& rhs) const { return !(*this == rhs); }

    bool operator<(const Lane&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Lane& a, Lane& b);

std::ostream& operator<<(std::ostream& out, const Lane& obj);

typedef struct _LaneGroup__isset {
    _LaneGroup__isset() : lanes_right(false), lanes_left(false) {}
    bool lanes_right : 1;
    bool lanes_left : 1;
} _LaneGroup__isset;

class LaneGroup : public virtual ::apache::thrift::TBase {
   public:
    LaneGroup(const LaneGroup&);
    LaneGroup& operator=(const LaneGroup&);
    LaneGroup() {}

    virtual ~LaneGroup() throw();
    Uuid id;
    std::vector<Uuid> lanes_right;
    std::vector<Uuid> lanes_left;

    _LaneGroup__isset __isset;

    void __set_id(const Uuid& val);

    void __set_lanes_right(const std::vector<Uuid>& val);

    void __set_lanes_left(const std::vector<Uuid>& val);

    bool operator==(const LaneGroup& rhs) const {
        if (!(id == rhs.id)) return false;
        if (!(lanes_right == rhs.lanes_right)) return false;
        if (!(lanes_left == rhs.lanes_left)) return false;
        return true;
    }
    bool operator!=(const LaneGroup& rhs) const { return !(*this == rhs); }

    bool operator<(const LaneGroup&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(LaneGroup& a, LaneGroup& b);

std::ostream& operator<<(std::ostream& out, const LaneGroup& obj);

typedef struct _LaneObjectBase__isset {
    _LaneObjectBase__isset() : type(false), lg_id(false), l_ids(false), offset(false) {}
    bool type : 1;
    bool lg_id : 1;
    bool l_ids : 1;
    bool offset : 1;
} _LaneObjectBase__isset;

class LaneObjectBase : public virtual ::apache::thrift::TBase {
   public:
    LaneObjectBase(const LaneObjectBase&);
    LaneObjectBase& operator=(const LaneObjectBase&);
    LaneObjectBase() : type((LaneObjectType::type)0), offset(0) {}

    virtual ~LaneObjectBase() throw();
    Uuid id;
    LaneObjectType::type type;
    Uuid lg_id;
    std::vector<Uuid> l_ids;
    double offset;

    _LaneObjectBase__isset __isset;

    void __set_id(const Uuid& val);

    void __set_type(const LaneObjectType::type val);

    void __set_lg_id(const Uuid& val);

    void __set_l_ids(const std::vector<Uuid>& val);

    void __set_offset(const double val);

    bool operator==(const LaneObjectBase& rhs) const {
        if (!(id == rhs.id)) return false;
        if (!(type == rhs.type)) return false;
        if (!(lg_id == rhs.lg_id)) return false;
        if (!(l_ids == rhs.l_ids)) return false;
        if (!(offset == rhs.offset)) return false;
        return true;
    }
    bool operator!=(const LaneObjectBase& rhs) const { return !(*this == rhs); }

    bool operator<(const LaneObjectBase&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(LaneObjectBase& a, LaneObjectBase& b);

std::ostream& operator<<(std::ostream& out, const LaneObjectBase& obj);

class Stopline : public virtual ::apache::thrift::TBase {
   public:
    Stopline(const Stopline&);
    Stopline& operator=(const Stopline&);
    Stopline() {}

    virtual ~Stopline() throw();
    LaneObjectBase lane_object;

    void __set_lane_object(const LaneObjectBase& val);

    bool operator==(const Stopline& rhs) const {
        if (!(lane_object == rhs.lane_object)) return false;
        return true;
    }
    bool operator!=(const Stopline& rhs) const { return !(*this == rhs); }

    bool operator<(const Stopline&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Stopline& a, Stopline& b);

std::ostream& operator<<(std::ostream& out, const Stopline& obj);

class Crosswalk : public virtual ::apache::thrift::TBase {
   public:
    Crosswalk(const Crosswalk&);
    Crosswalk& operator=(const Crosswalk&);
    Crosswalk() {}

    virtual ~Crosswalk() throw();
    LaneObjectBase lane_object;

    void __set_lane_object(const LaneObjectBase& val);

    bool operator==(const Crosswalk& rhs) const {
        if (!(lane_object == rhs.lane_object)) return false;
        return true;
    }
    bool operator!=(const Crosswalk& rhs) const { return !(*this == rhs); }

    bool operator<(const Crosswalk&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Crosswalk& a, Crosswalk& b);

std::ostream& operator<<(std::ostream& out, const Crosswalk& obj);

typedef struct _Parking__isset {
    _Parking__isset()
        : number_of_lots(false), parking_height(false), parking_width(false), line_width(false), outer_points(false) {}
    bool number_of_lots : 1;
    bool parking_height : 1;
    bool parking_width : 1;
    bool line_width : 1;
    bool outer_points : 1;
} _Parking__isset;

class Parking : public virtual ::apache::thrift::TBase {
   public:
    Parking(const Parking&);
    Parking& operator=(const Parking&);
    Parking() : number_of_lots(0), parking_height(0), parking_width(0), line_width(0) {}

    virtual ~Parking() throw();
    LaneObjectBase lane_object;
    int64 number_of_lots;
    double parking_height;
    double parking_width;
    double line_width;
    std::vector<std::vector<Point3D>> outer_points;

    _Parking__isset __isset;

    void __set_lane_object(const LaneObjectBase& val);

    void __set_number_of_lots(const int64 val);

    void __set_parking_height(const double val);

    void __set_parking_width(const double val);

    void __set_line_width(const double val);

    void __set_outer_points(const std::vector<std::vector<Point3D>>& val);

    bool operator==(const Parking& rhs) const {
        if (!(lane_object == rhs.lane_object)) return false;
        if (!(number_of_lots == rhs.number_of_lots)) return false;
        if (!(parking_height == rhs.parking_height)) return false;
        if (!(parking_width == rhs.parking_width)) return false;
        if (!(line_width == rhs.line_width)) return false;
        if (!(outer_points == rhs.outer_points)) return false;
        return true;
    }
    bool operator!=(const Parking& rhs) const { return !(*this == rhs); }

    bool operator<(const Parking&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Parking& a, Parking& b);

std::ostream& operator<<(std::ostream& out, const Parking& obj);

typedef struct _Roadsign__isset {
    _Roadsign__isset() : sign_type(false), position(false), rotation(false) {}
    bool sign_type : 1;
    bool position : 1;
    bool rotation : 1;
} _Roadsign__isset;

class Roadsign : public virtual ::apache::thrift::TBase {
   public:
    Roadsign(const Roadsign&);
    Roadsign& operator=(const Roadsign&);
    Roadsign() : sign_type(), rotation(0) {}

    virtual ~Roadsign() throw();
    LaneObjectBase lane_object;
    std::string sign_type;
    Point3D position;
    double rotation;

    _Roadsign__isset __isset;

    void __set_lane_object(const LaneObjectBase& val);

    void __set_sign_type(const std::string& val);

    void __set_position(const Point3D& val);

    void __set_rotation(const double val);

    bool operator==(const Roadsign& rhs) const {
        if (!(lane_object == rhs.lane_object)) return false;
        if (!(sign_type == rhs.sign_type)) return false;
        if (!(position == rhs.position)) return false;
        if (!(rotation == rhs.rotation)) return false;
        return true;
    }
    bool operator!=(const Roadsign& rhs) const { return !(*this == rhs); }

    bool operator<(const Roadsign&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(Roadsign& a, Roadsign& b);

std::ostream& operator<<(std::ostream& out, const Roadsign& obj);

class PointViz : public virtual ::apache::thrift::TBase {
   public:
    PointViz(const PointViz&);
    PointViz& operator=(const PointViz&);
    PointViz() : radius(0) {}

    virtual ~PointViz() throw();
    Point3D point;
    double radius;

    void __set_point(const Point3D& val);

    void __set_radius(const double val);

    bool operator==(const PointViz& rhs) const {
        if (!(point == rhs.point)) return false;
        if (!(radius == rhs.radius)) return false;
        return true;
    }
    bool operator!=(const PointViz& rhs) const { return !(*this == rhs); }

    bool operator<(const PointViz&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(PointViz& a, PointViz& b);

std::ostream& operator<<(std::ostream& out, const PointViz& obj);

class MapMessage : public virtual ::apache::thrift::TBase {
   public:
    MapMessage(const MapMessage&);
    MapMessage& operator=(const MapMessage&);
    MapMessage() : op((MessageOp::type)0) {}

    virtual ~MapMessage() throw();
    MessageOp::type op;
    MapContainer container;

    void __set_op(const MessageOp::type val);

    void __set_container(const MapContainer& val);

    bool operator==(const MapMessage& rhs) const {
        if (!(op == rhs.op)) return false;
        if (!(container == rhs.container)) return false;
        return true;
    }
    bool operator!=(const MapMessage& rhs) const { return !(*this == rhs); }

    bool operator<(const MapMessage&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(MapMessage& a, MapMessage& b);

std::ostream& operator<<(std::ostream& out, const MapMessage& obj);

class CarMessage : public virtual ::apache::thrift::TBase {
   public:
    CarMessage(const CarMessage&);
    CarMessage& operator=(const CarMessage&);
    CarMessage() : op((MessageOp::type)0), n_points_viz(0) {}

    virtual ~CarMessage() throw();
    MessageOp::type op;
    Pose pose;
    int64 n_points_viz;
    std::vector<PointViz> points_viz;

    void __set_op(const MessageOp::type val);

    void __set_pose(const Pose& val);

    void __set_n_points_viz(const int64 val);

    void __set_points_viz(const std::vector<PointViz>& val);

    bool operator==(const CarMessage& rhs) const {
        if (!(op == rhs.op)) return false;
        if (!(pose == rhs.pose)) return false;
        if (!(n_points_viz == rhs.n_points_viz)) return false;
        if (!(points_viz == rhs.points_viz)) return false;
        return true;
    }
    bool operator!=(const CarMessage& rhs) const { return !(*this == rhs); }

    bool operator<(const CarMessage&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(CarMessage& a, CarMessage& b);

std::ostream& operator<<(std::ostream& out, const CarMessage& obj);

class ThriftImage : public virtual ::apache::thrift::TBase {
   public:
    ThriftImage(const ThriftImage&);
    ThriftImage& operator=(const ThriftImage&);
    ThriftImage() : width(0), height(0), channels(0), bytes() {}

    virtual ~ThriftImage() throw();
    int16_t width;
    int16_t height;
    int8_t channels;
    std::string bytes;
    Pose pose;

    void __set_width(const int16_t val);

    void __set_height(const int16_t val);

    void __set_channels(const int8_t val);

    void __set_bytes(const std::string& val);

    void __set_pose(const Pose& val);

    bool operator==(const ThriftImage& rhs) const {
        if (!(width == rhs.width)) return false;
        if (!(height == rhs.height)) return false;
        if (!(channels == rhs.channels)) return false;
        if (!(bytes == rhs.bytes)) return false;
        if (!(pose == rhs.pose)) return false;
        return true;
    }
    bool operator!=(const ThriftImage& rhs) const { return !(*this == rhs); }

    bool operator<(const ThriftImage&) const;

    uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
    uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

    virtual void printTo(std::ostream& out) const;
};

void swap(ThriftImage& a, ThriftImage& b);

std::ostream& operator<<(std::ostream& out, const ThriftImage& obj);

}  // namespace map_thrift

#endif
