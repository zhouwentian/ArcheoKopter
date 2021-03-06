#ifndef _ISAE_FRAME_HPP_
#define _ISAE_FRAME_HPP_

/* this define allow to override type generated by simulink and to use the
 * proper definition */
#define _DEFINED_TYPEDEF_FOR_frame_t_

#include <cstdint>

namespace ISAE {

enum frame_t : std::uint8_t {
	GLOBAL_FRAME, /* global absolute frame, as provided by GPS for example */
	LOCAL_NED,    /* local frame (or inertial frame), Z points downwards */
	LOCAL_ENU,   /* local frame (or inertial frame), Z points upwards */
	BODY_NED,    /* body-fixed frame, Z points downwards */
	BODY_ENU     /* body-fixed frame, Z points upwards */
};

}

#endif /* _ISAE_FRAME_HPP_ */
