@addtogroup services
@{

The Services may be considered like servers and the modules/elements/entities which need
to use a service may be considered like clients.

A queue is used to dispatch messages between the clients and the servers.

Each service has to register on the queue (@ref cfw_register_service) and each "client" which wants
to communicate with a service has to register on the queue too (@ref cfw_init).

Once this is done, the "client" has to open the communication toward service it want (@ref cfw_open_service).


@}

