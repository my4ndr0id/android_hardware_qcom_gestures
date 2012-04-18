/*
Copyright (c) 2012, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/un.h>

#include "mm_sock.h"

/*===========================================================================
 * FUNCTION    - mm_socket_create -
 *
 * DESCRIPTION: opens a domain socket tied to camera ID and socket type
 *                        int cam_id: camera ID
  *                       mm_sock_type_t sock_type: socket type, TCP/UDP
 * retured fd related to the domain socket
 *==========================================================================*/
int mm_socket_create(char* path, mm_sock_type_t sock_type)
{
    int socket_fd;
    struct sockaddr_un sock_addr;
    int sktype;

    switch (sock_type)
    {
      case MM_SOCK_TYPE_UDP:
        sktype = SOCK_DGRAM;
        break;
      case MM_SOCK_TYPE_TCP:
        sktype = SOCK_STREAM;
        break;
      default:
        return -1;
    }
    socket_fd = socket(AF_UNIX, sktype, 0);
    if (socket_fd < 0) {
        return socket_fd;
    }

    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sun_family = AF_UNIX;
    snprintf(sock_addr.sun_path, UNIX_PATH_MAX, "%s", path);
    if (connect(socket_fd, (struct sockaddr *) &sock_addr,
      sizeof(sock_addr)) != 0)
      close(socket_fd);
      socket_fd = -1;

    return socket_fd;
}

/*===========================================================================
 * FUNCTION    - mm_socket_close -
 *
 * DESCRIPTION:  close domain socket by its fd
 *==========================================================================*/
void mm_socket_close(int fd)
{
    if (fd > 0) {
      close(fd);
    }
}

/*===========================================================================
 * FUNCTION    - mm_socket_sendmsg -
 *
 * DESCRIPTION:  send msg through domain socket
 *                         int fd: socket fd
 *                         void *msg: pointer to msg to be sent over domain socket
 *                         uint32_t buf_size: buffer size
 *                         int sendfd: file descriptors to be sent
 * return the total bytes of sent msg
 *==========================================================================*/
int mm_socket_sendmsg(
  int fd,
  void *msg,
  uint32_t buf_size,
  int sendfd)
{
    struct msghdr msgh;
    struct iovec iov[1];
    struct cmsghdr * cmsghp = NULL;
    char control[CMSG_SPACE(sizeof(int))];

    if (msg == NULL) {
      return -1;
    }
    memset(&msgh, 0, sizeof(msgh));
    msgh.msg_name = NULL;
    msgh.msg_namelen = 0;

    iov[0].iov_base = msg;
    iov[0].iov_len = buf_size;
    msgh.msg_iov = iov;
    msgh.msg_iovlen = 1;

    msgh.msg_control = NULL;
    msgh.msg_controllen = 0;

    // if sendfd is vlaid, we need to pass it through control msg
    if( sendfd > 0) {
      msgh.msg_control = control;
      msgh.msg_controllen = sizeof(control);
      cmsghp = CMSG_FIRSTHDR(&msgh);
      if (cmsghp != NULL) {
        cmsghp->cmsg_level = SOL_SOCKET;
        cmsghp->cmsg_type = SCM_RIGHTS;
        cmsghp->cmsg_len = CMSG_LEN(sizeof(int));
        *((int *)CMSG_DATA(cmsghp)) = sendfd;
      } else {
        return -1;
      }
    }

    return sendmsg(fd, &(msgh), 0);
}

/*===========================================================================
 * FUNCTION    - mm_socket_recvmsg -
 *
 * DESCRIPTION:  receive msg from domain socket.
 *                         int fd: socket fd
 *                         void *msg: pointer to mm_sock_msg_packet_t to hold incoming msg,
 *                                    need be allocated by the caller
 *                         uint32_t buf_size: the size of the buf that holds incoming msg
 *                         int *rcvdfd: pointer to hold recvd file descriptor if not NULL.
 * return the total bytes of received msg
 *==========================================================================*/
int mm_socket_recvmsg(
  int fd,
  void *msg,
  uint32_t buf_size,
  int *rcvdfd)
{
    struct msghdr msgh;
    struct iovec iov[1];
    struct cmsghdr *cmsghp = NULL;
    char control[CMSG_SPACE(sizeof(int))];
    int rcvd_fd = -1;
    int rcvd_len = 0;

    if ( (msg == NULL) || (buf_size <= 0) ) {
      return -1;
    }

    memset(&msgh, 0, sizeof(msgh));
    msgh.msg_name = NULL;
    msgh.msg_namelen = 0;
    msgh.msg_control = control;
    msgh.msg_controllen = sizeof(control);

    iov[0].iov_base = msg;
    iov[0].iov_len = buf_size;
    msgh.msg_iov = iov;
    msgh.msg_iovlen = 1;

    if ( (rcvd_len = recvmsg(fd, &(msgh), 0)) <= 0) {
      return rcvd_len;
    }

    if( ((cmsghp = CMSG_FIRSTHDR(&msgh)) != NULL) &&
		    (cmsghp->cmsg_len == CMSG_LEN(sizeof(int))) ) {
      if (cmsghp->cmsg_level == SOL_SOCKET &&
        cmsghp->cmsg_type == SCM_RIGHTS) {
        rcvd_fd = *((int *) CMSG_DATA(cmsghp));
      }
    }

    if (rcvdfd) {
      *rcvdfd = rcvd_fd;
    }

    return rcvd_len;
}

