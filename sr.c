#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12      /* the min sequence space for SR must be at least windowsize * 2 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt A_buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int A_windowfirst, A_windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int A_windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( A_windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    A_windowlast = (A_windowlast + 1) % WINDOWSIZE; 
    A_buffer[A_windowlast] = sendpkt;
    A_windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (A_windowcount == 1)
      starttimer(A,RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int ackcount = 0;
  int i;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* check if new ACK or duplicate */
    if (A_windowcount != 0) {
          int seqfirst = A_buffer[A_windowfirst].seqnum;
          int seqlast = A_buffer[A_windowlast].seqnum;
          /* check case when seqnum has and hasn't wrapped */
          if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
              ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast))) {

            /* packet is a new ACK */
            if (TRACE > 0)
              printf("----A: ACK %d is not a duplicate\n",packet.acknum);
            new_ACKs++;

            /* cumulative acknowledgement - determine how many packets are ACKed */
            if (packet.acknum >= seqfirst)
              ackcount = packet.acknum + 1 - seqfirst;
            else
              ackcount = SEQSPACE - seqfirst + packet.acknum;

	    /* slide window by the number of packets ACKed */
            A_windowfirst = (A_windowfirst + ackcount) % WINDOWSIZE;

            /* delete the acked packets from window buffer */
            for (i=0; i<ackcount; i++)
              A_windowcount--;

	    /* start timer again if there are still more unacked packets in window */
            stoptimer(A);
            if (A_windowcount > 0)
              starttimer(A, RTT);

          }
        }
        else
          if (TRACE > 0)
        printf ("----A: duplicate ACK received, do nothing!\n");
  }
  else 
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for(i=0; i<A_windowcount; i++) {

    if (TRACE > 0)
      printf ("---A: resending packet %d\n", (A_buffer[(A_windowfirst+i) % WINDOWSIZE]).seqnum);

    tolayer3(A, A_buffer[(A_windowfirst+i) % WINDOWSIZE]);
    packets_resent++;
    if (i==0) starttimer(A,RTT);
  }
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  A_windowfirst = 0;
  A_windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  A_windowcount = 0;
}



/********* Receiver (B)  variables and procedures ************/

static struct pkt B_buffer[WINDOWSIZE];  /* array for storing received packets before sending to application*/
static int B_windowfirst;    /* array indexes of the first/last packet in buffer */
static int B_windowcount;                /* the number of packets buffered */
static int B_expectedseqnum; /* sequence number associated with packet to go in windowfirst buffer slot*/


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seqfirst;
  int seqlast;
  int index;

  /* if not corrupted and received packet is in order */
  if  (!IsCorrupted(packet) ) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
    packets_received++;

    /* if within window, buffer packet / send packets to application and slide window */
    
    seqfirst = B_expectedseqnum;
    seqlast = (seqfirst + (WINDOWSIZE - 1)) % SEQSPACE;
    /* check case when seqnum has and hasn't wrapped */
    if (((seqfirst <= seqlast) && (packet.seqnum >= seqfirst && packet.seqnum <= seqlast)) ||
        ((seqfirst > seqlast) && (packet.seqnum >= seqfirst || packet.seqnum <= seqlast))) {

        int seqdistance = (packet.seqnum - B_expectedseqnum + SEQSPACE) % SEQSPACE;
        int bufferIndex = (B_windowfirst + seqdistance) % WINDOWSIZE;
        B_buffer[bufferIndex] = packet;
        B_windowcount++;

        /* Slide window for as many packets from windowstart until null packet */
        index = B_windowfirst;
        for (i = 0; i < WINDOWSIZE; i++){
          if (B_buffer[index].seqnum != NOTINUSE){
            tolayer5(B, B_buffer[index].payload);

            /* Packet has been delivered. Mark this as deleted from the buffer. */
            B_buffer[index].seqnum = NOTINUSE;

            B_windowfirst = (B_windowfirst + 1) % WINDOWSIZE;
            B_expectedseqnum = (B_expectedseqnum + 1) % SEQSPACE;
            B_windowcount--;
          } else {
            break;
          }

          index = (index + 1) % WINDOWSIZE;
        }
    }


    /* send an ACK for the received packet */
    sendpkt.acknum = packet.seqnum;
    sendpkt.seqnum = sendpkt.acknum;
      
    /* we don't have any data to send.  fill payload with 0's */
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = '0';  

    /* compute checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* send out packet */
    tolayer3 (B, sendpkt);
  }
  else {
  /* packet is corrupted or out of order */
    if (TRACE > 0) 
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
  }

  
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  struct pkt nullpkt;
  int i;

  B_windowfirst = 0;
  B_windowcount = 0;
  B_expectedseqnum = 0;
  
  nullpkt.seqnum = NOTINUSE;
  for (i = 0; i < WINDOWSIZE; i++){
    B_buffer[i] = nullpkt;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

