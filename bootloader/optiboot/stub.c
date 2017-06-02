/*
 * stub.c
 * Code for the GDB stub which should be in bootloader section.
 *
 *  Created on: 2. 6. 2017
 *      Author: jan dolinay
 */
static void handle_exception(void)
{
	uint8_t checksum, pkt_checksum;
	uint8_t b;

	gdb_ctx->singlestep_enabled = 0;		/* stepping by single instruction is enabled below for each step */

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
	/* Special case steeping after breakpoint... */
	if ( gdb_ctx->breakpoint_step ) {
		gdb_ctx->breakpoint_step = 0;
		gdb_update_breakpoints();
		gdb_disable_swinterrupt();
		return;
	}
#endif

	while (1) {
		b = getDebugChar();

		switch(b) {
		case '$':
			/* Read everything to buffer */
			gdb_ctx->buff_sz = 0;
			for (pkt_checksum = 0, b = getDebugChar();
				 b != '#'; b = getDebugChar())
			{
				gdb_ctx->buff[gdb_ctx->buff_sz++] = b;
				pkt_checksum += b;
			}
			gdb_ctx->buff[gdb_ctx->buff_sz] = 0;

			checksum  = hex2nib(getDebugChar()) << 4;
			checksum |= hex2nib(getDebugChar());

			/* send nack in case of wrong checksum  */
			if (pkt_checksum != checksum) {
				putDebugChar('-');
				continue;
			}

			/* ack */
			putDebugChar('+');

			/* parse already read buffer */
			if (gdb_parse_packet(gdb_ctx->buff))
				continue;

			if(gdb_ctx->singlestep_enabled || gdb_ctx->breakpoint_enabled)
			{
				/* this will generate interrupt after one instruction in main code */
				gdb_enable_swinterrupt();

			}
			else
			{
				gdb_disable_swinterrupt();
				/* Clear flag for the external INT. Added for no-timer flash BP. Probably not needed... */
				EIFR |= _BV(AVR8_SWINT_INTMASK);
			}

			/* leave the trap, continue execution */
			return;

		case '-':  /* NACK, repeat previous reply */
			gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);
			break;
		case '+':  /* ACK, great */
			break;
		case 0x03:
			/* user interrupt by Ctrl-C, send current state and
			   continue reading */
			gdb_send_state(GDB_SIGINT);
			break;
		default:
			gdb_send_reply(""); /* not supported */
			break;
		}
	}

}

