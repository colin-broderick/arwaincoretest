/***************************************************************************

	Header-only serial port reader which wraps boost::asio::serial_port.
	
	Author: CB, Greeve Ltd

***************************************************************************/

#ifndef _GREEVE_ARWAIN_SERIAL_H
#define _GREEVE_ARWAIN_SERIAL_H

#define BOOST_ALL_NO_LIB

#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class ArwainSerial
{
	public:
		const static int MAX_LINE_LEN = 512;
	
	private:
		enum flush_type { flush_receive = TCIFLUSH, flush_send = TCOFLUSH, flush_both = TCIOFLUSH };

	public:
		/** \brief Constructor. Opens and configures serial port.
		 * \param[in] com_port String defining the port (e.g. "COM14", "/dev/ttyS0").
		 * \param[in] baudrate Speed of the serial port (e.g. 9600, 115200).
		 */
		ArwainSerial(const std::string& com_port, const int baudrate)
		{
			this->port.open(com_port);
			this->port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
			this->port_open = true;
		}

		/** \brief Flush the serial port. */
		void flush()
		{
			::tcflush(this->port.lowest_layer().native_handle(), flush_both);
		}

		/** \brief Send a string to the serial port.
		 * \param message Content of the message; make sure to include EOL character(s) as required.
		 */
        void write(const std::string& message)
        {
            boost::asio::write(this->port, boost::asio::buffer(message.c_str(), message.size()));
        }

		/** \brief Reads a line from the serial port, up to a maximum length
		 * given by ArwainSerial::MAX_LINE_LEN.
		 * \return std::string as read from serial port.
		 */
		std::string readline()
		{
			// TODO This needs some timeout behaviour or it will block forever when nothing received on serial port.
			char buffer[ArwainSerial::MAX_LINE_LEN + 2];
			char c = 0;
			int index = 0;
			while (index < ArwainSerial::MAX_LINE_LEN)
			{
				boost::asio::read(this->port, boost::asio::buffer(&c, 1));
				if (c == '\r' || c  == '\n')
				{
					boost::asio::read(this->port, boost::asio::buffer(&c, 1));
					break;
				}
				buffer[index] = c;
				index++;
			}
			buffer[index] = 0;
			return std::string{buffer};
		}

		/** \brief Destructor. Ensures the port is closed before program exit. */
		~ArwainSerial()
		{
			this->close();
		}

	private:
		boost::asio::io_service io;
		boost::asio::serial_port port{io};
		bool port_open = false;

	private:
		/** \brief Close serial port handle. */
		void close()
		{
			if (this->port_open)
			{
				this->port_open = false;
				this->port.close();
			}
		}
};

#endif
