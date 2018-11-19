/*
 * File      : atomic.h
 */
#ifndef __ATOMIC_H__
#define __ATOMIC_H__

/**
 * @brief This macro should be called in the local variable
 * declarations section of any function which calls DISABLE_INTERRUPTS()
 * or RESTORE_INTERRUPTS().
 */
#define DECLARE_INTERRUPT_STATE uint32_t _primask
      
/**
 * @brief Disable interrupts, saving the previous state so it can be
 * later restored with RESTORE_INTERRUPTS().
 * \note Do not fail to call RESTORE_INTERRUPTS().
 * \note It is safe to nest this call.
 */
#define DISABLE_INTERRUPTS()                    \
do {                                          \
    _primask = __get_PRIMASK();            \
    __set_PRIMASK(1);                       \
} while(0)
  
  
/** 
 * @brief Restore the global interrupt state previously saved by
 * DISABLE_INTERRUPTS()
 * \note Do not call without having first called DISABLE_INTERRUPTS()
 * to have saved the state.
 * \note It is safe to nest this call.
*/
#define RESTORE_INTERRUPTS()      \
do {                            \
    __set_PRIMASK(_primask); \
} while(0)

/**
 * @brief Enable global interrupts without regard to the current or
 * previous state.
 */
#define INTERRUPTS_ON()               \
do {                                \
    __set_PRIMASK(0);                 \
} while(0)
  
/**
 * @brief Disable global interrupts without regard to the current or
 * previous state.
 */
#define INTERRUPTS_OFF()      \
do {                        \
    __set_PRIMASK(1);  \
} while(0)

/**
 * @brief A block of code may be made atomic by wrapping it with this
 * macro.  Something which is atomic cannot be interrupted by interrupts.
 */
#define ATOMIC(blah)         \
{                            \
    DECLARE_INTERRUPT_STATE;   \
    DISABLE_INTERRUPTS();      \
    { blah }                   \
    RESTORE_INTERRUPTS();      \
}

#define ENTER_CRITICAL(lock)    \
{   \
    lock = __get_PRIMASK();     \
    __set_PRIMASK(1);   \
}

#define EXIT_CRITICAL(lock)     \
{   \
    __set_PRIMASK(lock);    \
}

#endif
