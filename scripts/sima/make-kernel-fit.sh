#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
# make-kernel-fit.sh - build (and optionally sign) a U-Boot FIT for the SiMa
# Linux kernel, so U-Boot boots it via `bootm` in both secure and non-secure
# builds. Given a key the FIT is signed and a secure U-Boot (which carries the
# matching `required` key in its control DTB) enforces the signature; without a
# key it is unsigned and a non-secure U-Boot boots it unverified. A signed FIT
# still boots unverified on a non-secure U-Boot, so signing is purely additive.
#
# Only the kernel Image goes in the FIT. The device tree is deliberately left
# out so the board's existing DTB-load + overlay-apply flow is preserved; the
# boot script passes that externally loaded DTB to `bootm` as the fdt argument.
#
# The FIT must be staged at an address that does NOT overlap
# [load, load + image], or U-Boot aborts with "image overwritten". load/entry
# are emitted as two cells so the >32-bit load address survives.
#
# Kept in the kernel tree, with no bitbake dependencies, so a non-Yocto build
# can call it the same way.
#
# Usage:
#   make-kernel-fit.sh --image <Image> --out <fitImage> \
#       [--keydir <dir> --keyname <name>] [--algo sha256,rsa2048] \
#       [--mkimage <path>] [--load 0x1075000000]
set -eu

ALGO="sha256,rsa2048"
MKIMAGE="mkimage"
LOAD="0x1075000000"
IMAGE=""
OUT=""
KEYDIR=""
KEYNAME=""

die() { echo "make-kernel-fit.sh: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
	case "$1" in
		--image)   IMAGE="$2"; shift 2;;
		--out)     OUT="$2"; shift 2;;
		--keydir)  KEYDIR="$2"; shift 2;;
		--keyname) KEYNAME="$2"; shift 2;;
		--algo)    ALGO="$2"; shift 2;;
		--mkimage) MKIMAGE="$2"; shift 2;;
		--load)    LOAD="$2"; shift 2;;
		*) die "unknown argument: $1";;
	esac
done

[ -n "$IMAGE" ]   || die "--image is required"
[ -n "$OUT" ]     || die "--out is required"
[ -f "$IMAGE" ]   || die "kernel image not found: $IMAGE"

# The .its lives in a mktemp dir, and dtc resolves /incbin/ relative to the
# .its file's location, not the caller's cwd. Embed an absolute image path so
# the include resolves regardless of where this runs from.
case "$IMAGE" in
	/*) ;;
	*)  IMAGE="$(pwd)/$IMAGE";;
esac
[ -f "$IMAGE" ]   || die "kernel image not found after resolving path: $IMAGE"

# Signing is optional: --keydir and --keyname come as a pair (both -> signed,
# neither -> unsigned FIT). Anything in between is a mistake.
SIGN=0
if [ -n "$KEYDIR" ] || [ -n "$KEYNAME" ]; then
	[ -n "$KEYDIR" ]  || die "--keydir is required when --keyname is given"
	[ -n "$KEYNAME" ] || die "--keyname is required when --keydir is given"
	[ -f "$KEYDIR/$KEYNAME.key" ] || die "private key not found: $KEYDIR/$KEYNAME.key"
	SIGN=1
fi

# Split the (possibly >32-bit) load address into two 32-bit cells.
LOAD_HI="$(printf '0x%x' "$(( LOAD >> 32 ))")"
LOAD_LO="$(printf '0x%x' "$(( LOAD & 0xffffffff ))")"

ITS="$(mktemp)"
trap 'rm -f "$ITS"' EXIT

# When signing, the signature is over the *configuration* (required = "conf" on
# the U-Boot side), which covers the kernel image it references -- this prevents
# image mix-and-match. When unsigned, the config carries only the kernel ref.
{
	echo '/dts-v1/;'
	echo '/ {'
	echo '	description = "SiMa Modalix kernel FIT";'
	echo '	#address-cells = <2>;'
	echo ''
	echo '	images {'
	echo '		kernel {'
	echo '			description = "Linux kernel (arm64 Image)";'
	echo '			data = /incbin/("'"$IMAGE"'");'
	echo '			type = "kernel";'
	echo '			arch = "arm64";'
	echo '			os = "linux";'
	echo '			compression = "none";'
	echo "			load = <$LOAD_HI $LOAD_LO>;"
	echo "			entry = <$LOAD_HI $LOAD_LO>;"
	echo '			hash-1 { algo = "sha256"; };'
	echo '		};'
	echo '	};'
	echo ''
	echo '	configurations {'
	echo '		default = "conf";'
	echo '		conf {'
	echo '			description = "Linux kernel";'
	echo '			kernel = "kernel";'
	if [ "$SIGN" = 1 ]; then
		echo '			signature-1 {'
		echo "				algo = \"$ALGO\";"
		echo "				key-name-hint = \"$KEYNAME\";"
		echo '				sign-images = "kernel";'
		echo '			};'
	fi
	echo '		};'
	echo '	};'
	echo '};'
} > "$ITS"

if [ "$SIGN" = 1 ]; then
	"$MKIMAGE" -f "$ITS" -k "$KEYDIR" -r "$OUT"
else
	"$MKIMAGE" -f "$ITS" "$OUT"
fi
