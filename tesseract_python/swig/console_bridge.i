
%feature(director) console_bridge::OutputHandler;

namespace console_bridge
{
/** \brief The set of priorities for message logging */
enum LogLevel
  {
    CONSOLE_BRIDGE_LOG_DEBUG = 0,
    CONSOLE_BRIDGE_LOG_INFO,
    CONSOLE_BRIDGE_LOG_WARN,
    CONSOLE_BRIDGE_LOG_ERROR,
    CONSOLE_BRIDGE_LOG_NONE
  };

/**
 * \brief Set the minimum level of logging data to output. Messages with lower logging levels will
 * not be recorded.
 */
void setLogLevel(LogLevel level);

/**
 * \brief Retrieve the current level of logging data. Messages with lower logging levels will not be
 * recorded.
 */
LogLevel getLogLevel(void);

%pythondynamic OutputHandler;
class OutputHandler
{
public:
  OutputHandler(void){}

  virtual ~OutputHandler(void){}

  /**
   * \brief log a message to the output handler with the given text and logging level from a
   * specific file and line number
   * \param text to log
   * \param level console_bridge log level
   * \param filename of the output log
   * \param line
   */
  virtual void log(const std::string &text, LogLevel level, const char *filename, int line) = 0;
};

/**
 * \brief This function instructs console bridge that no messages should be outputted.
 * Equivalent to useOutputHandler(NULL)
 */
void noOutputHandler(void);

/**
 * \brief Restore the output handler that was previously in use (if any)
 */
void restorePreviousOutputHandler(void);

/**
 * \brief Specify the instance of the OutputHandler to use.
 * By default, this is OutputHandlerSTD
 */
void useOutputHandler(OutputHandler *oh);

/**
 * \brief Root level logging function.  This should not be invoked directly, but rather used via a
 * \ref logging "logging macro". Formats the message string given the arguments and forwards the
 * string to the output handler
 */
void log(const char *file,
                               int line,
                               LogLevel level,
                               const char* m
                               );

}