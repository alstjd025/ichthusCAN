/**
 * @file
 * This file declares a generic CANAdapter,
 * to facilitate frame transmission and reception.
 */

#ifndef CAN_ADAPTER_H
#define CAN_ADAPTER_H

#include <unistd.h>
#include <CANFrame.h>
#include <CANFrameParser.h>
#include <queue>
#include <mutex>

/**
 * Specifies the type of a CAN adapter
 */
typedef enum
{
    ADAPTER_NONE,
    ADAPTER_SOCKETCAN,
    ADAPTER_SLCAN,
    ADAPTER_LOGFILE,
} can_adapter_t;

/**
 * How a frame reception handler should look like
 */
typedef void (*pid_reception_handler_t)(can_frame_t*, std::queue<float>* velocity, std::mutex& qlock);
typedef void (*reception_handler_t)(can_frame_t*);

/**
 * Facilitates frame transmission and reception via a CAN adapter
 */
class CANAdapter
{
  protected:
    can_adapter_t adapter_type;

  public:
    /**
     * Pointer to a function which shall be called
     * when frames are being received from the CAN bus
     */
    pid_reception_handler_t pid_reception_handler;
    reception_handler_t reception_handler;
    /**
     * Pointer to a CAN frame parser object
     * which's parse_frame() method will be called when frames are received
     */
    CANFrameParser* parser = NULL;

    /** Constructor */
	CANAdapter();
	/** Destructor */
	virtual ~CANAdapter();

	/**
	 * Sends the referenced frame to the bus
	 */
	virtual void transmit(can_frame_t&);
};

#endif
