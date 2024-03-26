/*
 *  Created on: Apr 2023
 *  Modified by : Antonio Andara Mar, 2024
 * spins up a websocket and simple http server with some basic endpoints
 * and parameter parsing
 */

#include "server.h"


void print_server_app_header(void)
{
    xil_printf( "%20s %6d\r\n"
    		  , "basic http server"
			  , SERVER_PORT
			  );
}


void server_application_thread(void)
{
	int sock, new_sd;
	int size, n;
	struct sockaddr_in address, remote;
	char recv_buf[RECV_BUF_SIZE];
	char http_response[RESPONSE_SIZE];
	u32 reloads = 0;
	u8 button_val = 0;
	MotorParameters motor_parameters;

	memset(&address, 0, sizeof(address));

	// Create new socket
	if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0){
		return;
	}

	// Address settings
	address.sin_family = AF_INET;
	address.sin_port = htons(SERVER_PORT);
	address.sin_addr.s_addr = INADDR_ANY;

	// Bind socket to address
	if (lwip_bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0){
		return;
	}

	// Start listening for events on socket
	lwip_listen(sock, 0);
	size = sizeof(remote);

	struct pollfd fds[1];
	int ret;

	fds[0].fd = sock;
	fds[0].events = POLLIN;

	while(1){
		ret = poll(fds, 1, 10); // poll the socket for data
		if (ret > 0){
			new_sd = lwip_accept(sock, (struct sockaddr *)&remote, (socklen_t *)&size);
			memset(recv_buf, 0, sizeof(recv_buf));
			if ((n = read(new_sd, recv_buf, RECV_BUF_SIZE - 1)) < 0){
				xil_printf("%s: error reading from socket %d, closing socket\r\n", __FUNCTION__, new_sd);
				break;
			} else {
				motor_position = Stepper_getCurrentPositionInSteps();
				recv_buf[n] = '\0';
				if (strncmp(recv_buf, "GET /button-state", 17) == 0){
					xil_printf("inc: %lu\n", reloads);
					// Serve the button state
					snprintf(http_response, sizeof(http_response),
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: text/plain\r\n"
							"Connection: close\r\n\r\n"
							"%lu", reloads);
				} else if (strncmp(recv_buf, "GET /getStat", 12) == 0){
					reloads++;
					xQueueReceive(buttonStateQueue, &button_val, 0);
					// Construct the HTTP response with the HTML content
					snprintf(http_response, sizeof(http_response),
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: text/html\r\n"
							"Connection: close\r\n\r\n"
							"<!DOCTYPE html>"
							"<html><head><title>ECE 315</title>"
							"<script>"
							"function refresh() {"
							" setTimeout(function() {"
							" window.location.reload();"
							" }, 500);"
							"}"
							"window.onload = refresh;"
							"</script>"
							"</head>"
							"<body><h1>Simple HTTP server</h1>"
							"<p>Number of reloads: <input type='text' value='%lu' readonly></p>"
							"<p>Buttons state: <input type='text' value='%d' readonly></p>"
							"<p>Motor position: <input type='text' value='%lu' readonly></p>"
							"</body></html>", reloads, button_val, motor_position);
				} else if (strncmp(recv_buf, "GET / ", 6) == 0){
					snprintf(http_response, sizeof(http_response),
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: text/html\r\n"
							"Connection: close\r\n\r\n"
							"<!DOCTYPE html>"
							"<html><head><title>ECE 315</title>"
							"</head>"
							"<body><h1>Simple HTTP server</h1>"
							"<form action='/setParams'>"
							"<input type='text' name='rs' value='rotational_speed'/><br>"
							"<input type='text' name='ra' value='rotational_acceleration'><br>"
							"<input type='text' name='rd' value='rotational_deceleration'><br>"
							"<input type='text' name='cis' value='currentposition_in_steps'><br>"
							"<input type='text' name='fis' value='final_position_in_steps'><br>"
							"<input type='submit' value='submit'/><br>"
							"</form>"
							"</body></html>");
				} else if (strncmp(recv_buf, "GET /setParams", 14) == 0){
					xil_printf("recv:\n%s\n", recv_buf);
					process_query_string(recv_buf, &motor_parameters);
					xQueueSend(xQueue_FIFO1, &motor_parameters, 0);
					// TODO 1: Update form values with actual parameters from the queue
					// After receiving motor parameters through the query string and sending them to the queue,
					// you need to update the HTTP response form with the actual values sent to the queue.
					// Replace all the parameters in the snprintf call below with
					// the values obtained from the motor_parameters struct.
					snprintf(http_response, sizeof(http_response),
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: text/html\r\n"
							"Connection: close\r\n\r\n"
							"<!DOCTYPE html>"
							"<html><head><title>ECE 315</title>"
							"</head>"
							"<body><h1>Simple HTTP server</h1>"
							"<form action='/setParams'>"
							"<input type='text' name='rs' value='rotational_speed'/><br>"
							"<input type='text' name='ra' value='rotational_acceleration'><br>"
							"<input type='text' name='rd' value='rotational_deceleration'><br>"
							"<input type='text' name='cis' value='currentposition_in_steps'><br>"
							"<input type='text' name='fis' value='final_position_in_steps'><br>"
							"<input type='submit' value='submit'/><br>"
							"</form>"
							"</body></html>");
				}
				write_to_socket(new_sd, http_response);
			}
			close(new_sd);
		}
	}

}


// Helper function to write to socket
int write_to_socket(int sd, const char* buffer)
{
    int nwrote;

    nwrote = write(sd, buffer, strlen(buffer));

    if (nwrote < 0){
        xil_printf("%s: ERROR responding to client request. received = %d, written = %d\r\n",
                    __FUNCTION__, strlen(buffer), nwrote);
        xil_printf("Closing socket %d\r\n", sd);
    }
    return nwrote;
}


void process_query_string(const char* query, MotorParameters* params){
    char name[64];
	char value[64];
    const char* queryParamsStart;

    queryParamsStart = strchr(query, '?');

    if (!queryParamsStart){
            xil_printf("No query parameters found.\n");
            return;
	}
	queryParamsStart++;

    while (1){
        int bytesRead;
        if (sscanf(queryParamsStart, "%63[^=]=%63[^& ]%n", name, value, &bytesRead) == 2){
            parse_query_parameter(name, value, params);
			// Advance the pointer
			queryParamsStart += bytesRead;
			// Look for the next parameter, skipping '&'
			if (*queryParamsStart == '&'){
				queryParamsStart++;
			}
			// If end of string, break out of the loop
			if (*queryParamsStart == '\0'){
				break;
			}
        } else {
            break;
        }
    }
}


// Helper function to parse individual query parameter
int parse_query_parameter(const char* name, const char* value, MotorParameters* params)
{
	if (strcmp(name, "rs") == 0){
		params->rotational_speed = atof(value);
	} else if (strcmp(name, "ra") == 0){
		params->rotational_acceleration = atof(value);
	} else if (strcmp(name, "rd") == 0){
		params->rotational_deceleration = atof(value);
	} else if (strcmp(name, "cis") == 0){
		params->currentposition_in_steps = atol(value);
	} else if (strcmp(name, "fis") == 0){
				params->final_position = atol(value);
	} else {
		return 0; // Unrecognized parameter name
	}
	return 1; // Success
}
