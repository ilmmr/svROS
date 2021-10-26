### Concept

Here you can find the creation of two nodes that publish data through the same topic. Then the subscriber needs to filter this information, by checking the published data and filtering which publisher should get 'ahead'.

In order to get this right, it was created a new interface that not only deals with the message that is being passed on, but also deals with the information about the publisher node.
So, this process was divided in the creation for the interface, explained at `interfaces`, and the creation of the package related to the code of each node (at `src`), that later uses the interface as the new message type.
