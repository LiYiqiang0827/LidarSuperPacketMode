#!/usr/bin/python

import argparse
import os
import subprocess
import sys
import boto3
from aliyunsdkcore import client
from aliyunsdksts.request.v20150401 import AssumeRoleRequest
import oss2
import json

if sys.version_info[0] >= 3:
    from urllib.request import urlretrieve
    import urllib.request
else:
    from urllib import urlretrieve

my_path = os.path.dirname(os.path.realpath(__file__))

def list_tags(keys, local):
    if local:
        cmd = "git log --tags --simplify-by-decoration --pretty=\"format:%ci %d\""
    else:
        cmd = "git ls-remote --tags origin | sed -n 's_^.*/\([^/}]*\)$_\\1_p' "
    # print(cmd)
    lines = subprocess.check_output(cmd, shell=True)
    lines = lines.splitlines()
    for line in lines:
        not_print = False
        for key in keys:
            if key not in line:
                not_print = True
                break
        if not_print:
            # print("not match: {}".format(line))
            pass
        else:
            print(line)
    return 0

def get_git_commit_id(commit_id):
    try:
        commit_id = subprocess.check_output(["git", "rev-parse", commit_id]).strip()
        if sys.version_info[0] >= 3:
            commit_id = commit_id.decode("utf-8")

    except Exception:
        print("cannot resolve commit_id {}".format(commit_id))
        exit(1)
    print("use commit {}".format(commit_id))
    return commit_id

def extract_sdk_file(file):
    my_path = os.path.dirname(os.path.realpath(__file__))
    target_path = os.path.join(my_path, "../sdk")
    if not os.path.exists(target_path):
        os.makedirs(target_path)
    import subprocess
    print subprocess.check_output(['tar', 'xzvf', file, "--directory", target_path, "--overwrite"])

def do_fetch_file(file, file_url, bucket, base, force_download, storage):
    download = True
    if os.path.isfile(file):
        print("{} is already there".format(file))
        if force_download:
            print("force download")
        else:
            statinfo = os.stat(file)
            if statinfo.st_size < 100000:
                print("file {} is invalid: {}, force download".format(file, statinfo.st_size))
            else:
                download = False
    if download:
        print("download {} from {}".format(file, file_url))
        if sys.version_info[0] >= 3:
            import requests
            r = requests.get(file_url)
            with open(file, "wb") as code:
                code.write(r.content)
        else:
            urlretrieve(file_url, file)
            pass
        if os.path.isfile(file):
            statinfo = os.stat(file)
        else:
            statinfo = None
        if (not statinfo) or statinfo.st_size < 100000:
            print("file {} is invalid: {}, use boto to download".format(file,
                                                                        statinfo.st_size if statinfo else "Non"))
            if statinfo:
                os.remove(file)
            try:
                if storage == "s3":
                    S3.download_file(bucket, base, file)
                else:
                    OSS.download_file(bucket, base, file)
                statinfo = os.stat(file)
                if statinfo.st_size < 100000:
                    print("file {} is invalid: {}, cannot download".format(file))
                    os.remove(file)
            except Exception as e:
                print("*****\nfile {} DOES NOT EXIST in {}:{}\n***** \nerr={}".format(file, bucket, base, e))

def fetch_files(args, public_build, ros_version):
    if args.storage == "s3":
        server = "https://s3-us-west-2.amazonaws.com"
    else:
        server = "https://oss-cn-shanghai.aliyuncs.com"
    bucket_public = "iv-release"
    bucket_pipeline = "iv-bitbucket-pipeline"
    try:
        commit_num = int(args.tag, 16)
        commit_id = args.tag
    except Exception:
        if args.tag in ['HEAD']:
            commit_id  = args.tag
        elif args.tag == '' or args.tag.lower() == 'customer':
            if args.tag == '':
                latest_tag_file = 'LATEST_ENG_TAG'
                lastest_tag_url = "{}/{}/build/LATEST_ENG_TAG".format(server, bucket_public)
            else:
                latest_tag_file = 'LATEST_CUSTOMER_TAG'
                lastest_tag_url = "{}/{}/build/LATEST_CUSTOMER_TAG".format(server, bucket_public)
            if sys.version_info[0] >= 3:
                import requests
                r = requests.get(lastest_tag_url)
                with open(latest_tag_file, "wb") as l:
                    code.write(r.content)
            else:
                urlretrieve(lastest_tag_url, latest_tag_file)
            statinfo = os.stat(latest_tag_file)
            if statinfo.st_size < 1 or statinfo.st_size > 40:
                assert False, "Cannot get LATEST_TAG file, please check you network connection."
            with open(latest_tag_file) as f:
                first_line = f.readline()
                first_line = first_line.strip()
                args.tag = first_line
                commit_id = None
                print('########## Download {} {}'.format(latest_tag_file, args.tag))
        else:
            commit_id = None

    if public_build:
        ppp = 'public'
    else:
        ppp = 'internal'
    if commit_id is None:
        if public_build:
            bucket = bucket_public
            build = "release"
        else:
            bucket = bucket_pipeline
            build = "build"
        base = "{build}/TAG/{tag}".format(build=build, tag=args.tag)
        ver = args.tag
    else:
        assert ros_version == "melodic", "ros_version has to be melodic when use commit_id or HEAD"
        if len(commit_id) < 40:
            commit_id = get_git_commit_id(commit_id)
        bucket = bucket_pipeline
        base = "build/master/{commit_id}".format(server=server, commit_id=commit_id)
        ver = commit_id[:6]

    sdk_prefix = "inno-filter-perception" if args.filter else "inno-lidar-sdk"

    if args.filter:
        pass
    elif args.base_sdk is not None:
        sdk_prefix = "base-" + sdk_prefix
    else:
        for arch in ["", "arm-", "mingw64-"]:
            deb_file = "ros-{ros_version}-innovusion-driver-{ver}-{arch}{ppp}.deb".format(ros_version=ros_version,
                                                                                          ppp=ppp,
                                                                                          ver=ver,
                                                                                          arch=arch)
            deb_url = "{server}/{bucket}/{base}/{file}".format(server=server, bucket=bucket, base=base, file=deb_file)
            do_fetch_file(deb_file, deb_url, bucket, base, args.force_download, args.storage)

            if public_build:
                sdk_file = "{}-{ver}-{arch}public.extended.tgz".format(sdk_prefix, arch=arch, ver=ver)
                sdk_url = "{server}/{bucket}/{base}/{file}".format(server=server, bucket=bucket, base=base, file=sdk_file)
                do_fetch_file(sdk_file, sdk_url, bucket, base, args.force_download, args.storage)

    if args.base_sdk is None:
        if args.arch is None:
            archs = ["arm-", "qnx-", "1404-", "mingw64-", ""]
        elif args.arch == "1604":
            archs = [""]
        else:
            archs = [args.arch + "-"]
    elif args.base_sdk == "1604":
        archs = [""]
    else:
        archs = [args.base_sdk + "-"]
    for arch in archs:
        for prefix in [sdk_prefix, "inno-lidar-embedded"]:
            print("###################", prefix)
            sdk_file = "{}-{ver}-{arch}{ppp}.tgz".format(prefix, ppp=ppp, arch=arch, ver=ver)
            sdk_url = "{server}/{bucket}/{base}/{file}".format(server=server, bucket=bucket, base=base, file=sdk_file)
            do_fetch_file(sdk_file, sdk_url, bucket, base, args.force_download, args.storage)

    if len(archs) == 1 and (args.base_sdk or args.filter) and args.extract_base:
        extract_sdk_file(sdk_file)

    return

def main(args):
    if isinstance(args.list_tags, list):
        return list_tags(args.list_tags, False)
    if isinstance(args.list_tags_local, list):
        return list_tags(args.list_tags_local, True)

    if args.ros_version == "all":
        fetch_files(args, not args.internal, "noetic")
        fetch_files(args, not args.internal, "melodic")
        fetch_files(args, not args.internal, "kinetic")
    else:
        fetch_files(args, not args.internal, args.ros_version);


    return 0

class S3:
    def __init__(self, session, bucket):
        print("boto3 version", boto3.__version__)
        self.bucket = bucket
        try:
            self.client = session.client('s3')
        except Exception as err:
            print("Failed to create boto3 client use profle {}".format(err))
            sys.exit(1)

    def is_file_exist(self, key):
        from botocore.exceptions import ClientError
        try:
            self.client.head_object(Bucket=self.bucket, Key=key)
            return True
        except ClientError as e:
            print(e, key, self.bucket)
            return False
        except Exception as e:
            print(e, key, self.bucket)
            return False
        print("should not reach here")
        return False

    def download(self, local_file, key):
        from botocore.exceptions import ClientError
        try:
            if not self.is_file_exist(key):
                return False
            with open(local_file, 'wb') as data:
                self.client.download_fileobj(Bucket=self.bucket, Key=key, Fileobj=data)
            print("{} downloaded".format(local_file))
        except ClientError as err:
            print("Failed to download file from S3 {}. {}".format(key, str(err)))
            return False
        except IOError as err:
            print("Failed to access {} in this directory. {}".format(local_file, str(err)))
            return False
        except Exception as err:
            print("Failed to download {} and save to {}. {}".format(key, local_file, str(err)))
            return False;
        return True

    @staticmethod
    def download_file(bucket, base, file):
        profile_name = "sw_release_ro"
        try:
            session = boto3.Session(profile_name=profile_name)
        except:
            session = boto3.Session()
        s = S3(session, bucket)
        s.download(file, os.path.join(base, file))


class OSS:
    def __init__(self, region, endpoint, bucket, ak_id, ak_secret, role_arn, policy):
        try:
            clt = client.AcsClient(ak_id, ak_secret, region)
            req = AssumeRoleRequest.AssumeRoleRequest()
            req.set_accept_format('json')
            req.set_RoleArn(role_arn)
            req.set_RoleSessionName('session-sts')
            req.set_Policy(policy)
            body = clt.do_action_with_exception(req)
            token = json.loads(oss2.to_unicode(body))
            auth = oss2.StsAuth(token['Credentials']['AccessKeyId'],
                                token['Credentials']['AccessKeySecret'],
                                token['Credentials']['SecurityToken'])
            self.bucket = oss2.Bucket(auth, endpoint, bucket)
        except Exception as err:
            print("Failed to init OSS session: {}".format(str(err)))
            sys.exit(1)

    def download(self, local_file, file):
        try:
            print("Download {} from OSS".format(file))
            self.bucket.get_object_to_file(file, local_file)
        except Exception as err:
            print("Failed to download file {} from OSS: {}".format(file, str(err)))
            return False
        return True

    @staticmethod
    def download_file(bucket, base, file):
        region = "cn-shanghai"
        endpoint = "oss-cn-shanghai.aliyuncs.com"
        access_key_id = os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"]
        access_key_secret = os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"]
        role_arn = 'acs:ram::1178134037455340:role/ivsdkaccesspublic'
        policy_text = '{\
                            "Version": "1",\
                            "Statement": [\
                                {\
                                    "Effect": "Allow",\
                                    "Action": [\
                                        "oss:GetObject"\
                                    ],\
                                    "Resource": "acs:oss:*:1178134037455340:iv-sdk-bucket/*"\
                                }\
                            ]\
                        }'.replace("iv-sdk-bucket", bucket)
        oss = OSS(region, endpoint, bucket, access_key_id, access_key_secret, role_arn, policy_text)
        oss.download(file, os.path.join(base, file))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", type=str, default="", help="git repo tag or commit-id")
    parser.add_argument("--ros-version", type=str, default="all",
                        choices=["all", "kinetic", "melodic", "noetic"],
                        help="ros version, default is all")
    parser.add_argument("-s", "--show-sample", action="store_true", help="show sample", default=False)
    parser.add_argument("-f", "--filter", action="store_true", help="download filter", default=False)
    parser.add_argument("--base-sdk", type=str, choices=["arm", "qnx", "1404", "1604"], default=None)
    parser.add_argument("--arch", type=str, choices=["arm", "qnx", "1404", "1604"], default=None)
    parser.add_argument("--internal", action="store_true", help="internal build", default=False)
    parser.add_argument("--extract-base", action="store_true", help="extract base sdk", default=False)
    parser.add_argument("--force-download", action="store_true", help="force download", default=False)
    parser.add_argument("--list-tags", metavar="KEY", nargs='*', type=str, default=None, help="list remote tags that contains keys")
    parser.add_argument("--list-tags-local", metavar="KEY", nargs='*', type=str, default=None, help="list local tags that contains keys")
    parser.add_argument("--storage", type=str, choices=["s3", "oss"], default="s3", help="which storage from to download")
    args = parser.parse_args()
    if args.arch == "1404":
        args.arch = "1604"
    if args.show_sample:
        sample = '''
- see all command line options:
  {cmd} -h

- download latest stable eng release:
  {cmd}

- download latest stable customer release:
  {cmd} --tag customer

- download a release tagged as release-1.1.2-rc0:
  {cmd} --tag release-1.1.2-rc0

- download filter release tagged as release-1.6.2-rc3 for x86:
  {cmd} --tag release-1.6.1-rc3 --filter --arch 1604

- see all release tags:
  {cmd} --list-tags

        '''
        print(sample.format(cmd=sys.argv[0]))
        exit(0)

    main(args)
