from __future__ import print_function
import os
import sys
import argparse
from aliyunsdkcore import client
from aliyunsdksts.request.v20150401 import AssumeRoleRequest
import oss2
import json


def upload_to_oss(bucket, artifact, bucket_key):
    region = "cn-shanghai"
    endpoint = "oss-cn-shanghai.aliyuncs.com"
    print("Upload {} to https://{}/{}/{}".format(artifact, endpoint, bucket, bucket_key))
    access_key_id = os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"]
    access_key_secret = os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"]
    role_arn = 'acs:ram::1178134037455340:role/ivsdkaccess'
    policy_text = '{\
                        "Version": "1",\
                        "Statement": [\
                            {\
                                "Effect": "Allow",\
                                "Action": [\
                                    "oss:PutObject",\
                                    "oss:PostObject"\
                                ],\
                                "Resource": "acs:oss:*:1178134037455340:iv-sdk-bucket/*"\
                            }\
                        ]\
                    }'.replace("iv-sdk-bucket", bucket)
    try:
        clt = client.AcsClient(access_key_id, access_key_secret, region)
        req = AssumeRoleRequest.AssumeRoleRequest()
        req.set_accept_format('json')
        req.set_RoleArn(role_arn)
        req.set_RoleSessionName('session-sts')
        req.set_Policy(policy_text)
        body = clt.do_action_with_exception(req)
        token = json.loads(oss2.to_unicode(body))
        auth = oss2.StsAuth(token['Credentials']['AccessKeyId'],
                            token['Credentials']['AccessKeySecret'],
                            token['Credentials']['SecurityToken'])
        bucket = oss2.Bucket(auth, endpoint, bucket, connect_timeout=30)
        bucket.put_object_from_file(bucket_key, artifact)
    except Exception as err:
        print("Failed to upload: {}", str(err))
        return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bucket", help="Name of the existing oss bucket")
    parser.add_argument("--artifact", nargs='+', help="Names of the artifact to be uploaded to oss")
    parser.add_argument("--bucket_key", help="Name of the oss Bucket key")
    args = parser.parse_args()

    print("total files to upload {}".format(len(args.artifact)))
    for f in args.artifact:
        if not upload_to_oss(args.bucket, f, args.bucket_key + '/' + os.path.basename(f)):
            sys.exit(1)


if __name__ == "__main__":
    main()
