# homework-1-f2017-benbdon
## 1. The single node contains one publisher and one subscriber. Answer the following questions and describe how you found the answers.
### (a) What are the names of the corresponding topics?
####    `/demo_publish_topic`
####    `/demo_suscriber_topic`
### b) Describe the message definition for each of the topics.
####    `/demo_publish_topic` has a custom message definition `/ME495Pub`
####    `/demo_suscriber_topic` uses a standard message of type `string`
### (c) What packages define the messages?
####    `ME495Pub` and `std_msgs` packes define the `me495_hw1` message
####    `std_msgs` package define the 'String' message
### (d) What frequency (approximately) does the publisher publish data at?
####    `demo_publish_topic` publishes at ~50 Hz
### (e) Use rqt plot to plot the data being published. This data should be familiar, what is it?
####    It appears to be a sine wave between -10 and 10.
### (f) Publish a message on the subscriber’s topic to trigger a callback in the node. What happens in the terminal where the node is running?
####    The node stops running and publishes the following `[ INFO] [timestamp]: Manipulated String: <input reversed>` For example, I provided the words `test` and got back `tset`. Likewise I published `backwards` and got back `sdrawkcab`.
## 2. The node also contains a single service provider. Answer the following questions and describe how you found the answers.
### (a) What is the name of the service?
####    `ME495Srv`
### (b) Describe the definition of the service request and response.
####    The service requests a `uint32 input` and responds with a `uint8 output`
### (c) This service performs some sort of mathematical computation. Describe this computation.
####    It appears this service counts the number of digits provided as an input.
## 3. This node [alt_figure_8](src/alt_figure_8.py) commands turtlesim to drive in a figure-8 and its equations of motion are based on the equations provided on page 13 in "Control of Wheeled Mobile Robots: An Experimental Overview" by Alessandro De Luca, Giuseppe Oriolo, Marilena Vendittelli, Dipartimento di Informatica e Sistemistica, Università degli Studi di Roma “LaSapienza”, Italy. This version makes no use of feedback control. 
