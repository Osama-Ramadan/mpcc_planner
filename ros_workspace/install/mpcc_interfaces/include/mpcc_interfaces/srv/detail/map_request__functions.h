// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mpcc_interfaces:srv/MapRequest.idl
// generated code does not contain a copyright notice

#ifndef MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__FUNCTIONS_H_
#define MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mpcc_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "mpcc_interfaces/srv/detail/map_request__struct.h"

/// Initialize srv/MapRequest message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mpcc_interfaces__srv__MapRequest_Request
 * )) before or use
 * mpcc_interfaces__srv__MapRequest_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Request__init(mpcc_interfaces__srv__MapRequest_Request * msg);

/// Finalize srv/MapRequest message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Request__fini(mpcc_interfaces__srv__MapRequest_Request * msg);

/// Create srv/MapRequest message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mpcc_interfaces__srv__MapRequest_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
mpcc_interfaces__srv__MapRequest_Request *
mpcc_interfaces__srv__MapRequest_Request__create();

/// Destroy srv/MapRequest message.
/**
 * It calls
 * mpcc_interfaces__srv__MapRequest_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Request__destroy(mpcc_interfaces__srv__MapRequest_Request * msg);

/// Check for srv/MapRequest message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Request__are_equal(const mpcc_interfaces__srv__MapRequest_Request * lhs, const mpcc_interfaces__srv__MapRequest_Request * rhs);

/// Copy a srv/MapRequest message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Request__copy(
  const mpcc_interfaces__srv__MapRequest_Request * input,
  mpcc_interfaces__srv__MapRequest_Request * output);

/// Initialize array of srv/MapRequest messages.
/**
 * It allocates the memory for the number of elements and calls
 * mpcc_interfaces__srv__MapRequest_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Request__Sequence__init(mpcc_interfaces__srv__MapRequest_Request__Sequence * array, size_t size);

/// Finalize array of srv/MapRequest messages.
/**
 * It calls
 * mpcc_interfaces__srv__MapRequest_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Request__Sequence__fini(mpcc_interfaces__srv__MapRequest_Request__Sequence * array);

/// Create array of srv/MapRequest messages.
/**
 * It allocates the memory for the array and calls
 * mpcc_interfaces__srv__MapRequest_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
mpcc_interfaces__srv__MapRequest_Request__Sequence *
mpcc_interfaces__srv__MapRequest_Request__Sequence__create(size_t size);

/// Destroy array of srv/MapRequest messages.
/**
 * It calls
 * mpcc_interfaces__srv__MapRequest_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Request__Sequence__destroy(mpcc_interfaces__srv__MapRequest_Request__Sequence * array);

/// Check for srv/MapRequest message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Request__Sequence__are_equal(const mpcc_interfaces__srv__MapRequest_Request__Sequence * lhs, const mpcc_interfaces__srv__MapRequest_Request__Sequence * rhs);

/// Copy an array of srv/MapRequest messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Request__Sequence__copy(
  const mpcc_interfaces__srv__MapRequest_Request__Sequence * input,
  mpcc_interfaces__srv__MapRequest_Request__Sequence * output);

/// Initialize srv/MapRequest message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mpcc_interfaces__srv__MapRequest_Response
 * )) before or use
 * mpcc_interfaces__srv__MapRequest_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Response__init(mpcc_interfaces__srv__MapRequest_Response * msg);

/// Finalize srv/MapRequest message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Response__fini(mpcc_interfaces__srv__MapRequest_Response * msg);

/// Create srv/MapRequest message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mpcc_interfaces__srv__MapRequest_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
mpcc_interfaces__srv__MapRequest_Response *
mpcc_interfaces__srv__MapRequest_Response__create();

/// Destroy srv/MapRequest message.
/**
 * It calls
 * mpcc_interfaces__srv__MapRequest_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Response__destroy(mpcc_interfaces__srv__MapRequest_Response * msg);

/// Check for srv/MapRequest message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Response__are_equal(const mpcc_interfaces__srv__MapRequest_Response * lhs, const mpcc_interfaces__srv__MapRequest_Response * rhs);

/// Copy a srv/MapRequest message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Response__copy(
  const mpcc_interfaces__srv__MapRequest_Response * input,
  mpcc_interfaces__srv__MapRequest_Response * output);

/// Initialize array of srv/MapRequest messages.
/**
 * It allocates the memory for the number of elements and calls
 * mpcc_interfaces__srv__MapRequest_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Response__Sequence__init(mpcc_interfaces__srv__MapRequest_Response__Sequence * array, size_t size);

/// Finalize array of srv/MapRequest messages.
/**
 * It calls
 * mpcc_interfaces__srv__MapRequest_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Response__Sequence__fini(mpcc_interfaces__srv__MapRequest_Response__Sequence * array);

/// Create array of srv/MapRequest messages.
/**
 * It allocates the memory for the array and calls
 * mpcc_interfaces__srv__MapRequest_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
mpcc_interfaces__srv__MapRequest_Response__Sequence *
mpcc_interfaces__srv__MapRequest_Response__Sequence__create(size_t size);

/// Destroy array of srv/MapRequest messages.
/**
 * It calls
 * mpcc_interfaces__srv__MapRequest_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
void
mpcc_interfaces__srv__MapRequest_Response__Sequence__destroy(mpcc_interfaces__srv__MapRequest_Response__Sequence * array);

/// Check for srv/MapRequest message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Response__Sequence__are_equal(const mpcc_interfaces__srv__MapRequest_Response__Sequence * lhs, const mpcc_interfaces__srv__MapRequest_Response__Sequence * rhs);

/// Copy an array of srv/MapRequest messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mpcc_interfaces
bool
mpcc_interfaces__srv__MapRequest_Response__Sequence__copy(
  const mpcc_interfaces__srv__MapRequest_Response__Sequence * input,
  mpcc_interfaces__srv__MapRequest_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MPCC_INTERFACES__SRV__DETAIL__MAP_REQUEST__FUNCTIONS_H_
