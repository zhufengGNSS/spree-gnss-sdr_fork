/*
 * Generated by asn1c-0.9.22 (http://lionet.info/asn1c)
 * From ASN.1 module "RRLP-Components"
 * 	found in "../rrlp-components.asn"
 */

#include "GPS-MsrElement.h"

static int
memb_cNo_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 63)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_doppler_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -32768 && value <= 32767)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_wholeChips_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 1022)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_fracChips_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 1024)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_pseuRangeRMSErr_constraint_1(asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 63)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		_ASN_CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_per_constraints_t ASN_PER_MEMB_C_NO_CONSTR_3 = {
	{ APC_CONSTRAINED,	 6,  6,  0,  63 }	/* (0..63) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t ASN_PER_MEMB_DOPPLER_CONSTR_4 = {
	{ APC_CONSTRAINED,	 16,  16, -32768,  32767 }	/* (-32768..32767) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t ASN_PER_MEMB_WHOLE_CHIPS_CONSTR_5 = {
	{ APC_CONSTRAINED,	 10,  10,  0,  1022 }	/* (0..1022) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t ASN_PER_MEMB_FRAC_CHIPS_CONSTR_6 = {
	{ APC_CONSTRAINED,	 11,  11,  0,  1024 }	/* (0..1024) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_per_constraints_t ASN_PER_MEMB_PSEU_RANGE_RMS_ERR_CONSTR_8 = {
	{ APC_CONSTRAINED,	 6,  6,  0,  63 }	/* (0..63) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_GPS_MsrElement_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, satelliteID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SatelliteID,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"satelliteID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, cNo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_cNo_constraint_1,
		&ASN_PER_MEMB_C_NO_CONSTR_3,
		0,
		"cNo"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, doppler),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_doppler_constraint_1,
		&ASN_PER_MEMB_DOPPLER_CONSTR_4,
		0,
		"doppler"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, wholeChips),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_wholeChips_constraint_1,
		&ASN_PER_MEMB_WHOLE_CHIPS_CONSTR_5,
		0,
		"wholeChips"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, fracChips),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_fracChips_constraint_1,
		&ASN_PER_MEMB_FRAC_CHIPS_CONSTR_6,
		0,
		"fracChips"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, mpathIndic),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_MpathIndic,
		0,	/* Defer constraints checking to the member type */
		0,	/* No PER visible constraints */
		0,
		"mpathIndic"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GPS_MsrElement, pseuRangeRMSErr),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		memb_pseuRangeRMSErr_constraint_1,
		&ASN_PER_MEMB_PSEU_RANGE_RMS_ERR_CONSTR_8,
		0,
		"pseuRangeRMSErr"
		},
};
static ber_tlv_tag_t asn_DEF_GPS_MsrElement_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_GPS_MsrElement_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* satelliteID at 466 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* cNo at 467 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* doppler at 468 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* wholeChips at 469 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* fracChips at 470 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* mpathIndic at 474 */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 } /* pseuRangeRMSErr at 475 */
};
static asn_SEQUENCE_specifics_t asn_SPC_GPS_MsrElement_specs_1 = {
	sizeof(struct GPS_MsrElement),
	offsetof(struct GPS_MsrElement, _asn_ctx),
	asn_MAP_GPS_MsrElement_tag2el_1,
	7,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_GPS_MsrElement = {
	"GPS-MsrElement",
	"GPS-MsrElement",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	SEQUENCE_decode_uper,
	SEQUENCE_encode_uper,
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_GPS_MsrElement_tags_1,
	sizeof(asn_DEF_GPS_MsrElement_tags_1)
		/sizeof(asn_DEF_GPS_MsrElement_tags_1[0]), /* 1 */
	asn_DEF_GPS_MsrElement_tags_1,	/* Same as above */
	sizeof(asn_DEF_GPS_MsrElement_tags_1)
		/sizeof(asn_DEF_GPS_MsrElement_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_GPS_MsrElement_1,
	7,	/* Elements count */
	&asn_SPC_GPS_MsrElement_specs_1	/* Additional specs */
};
